/**
 ******************************************************************************
 * @file    magnetometer.c
 * @brief   Board-level driver for the MLX90393 3-axis magnetometer.
 ******************************************************************************
 */

#include "magnetometer.h"

/* Command bytes (datasheet Table 11, "Command List"). zyxt = 1111 selects
 * temperature + all three axes. */
#define MAG_CMD_SM_TXYZ 0x3Fu /* Start Single Measurement Mode, T+X+Y+Z */
#define MAG_CMD_RM_TXYZ 0x4Fu /* Read Measurement, T+X+Y+Z */
#define MAG_CMD_EXIT 0x80u
#define MAG_CMD_RR 0x50u /* Read Register (abc = 0) */
#define MAG_CMD_WR 0x60u /* Write Register (abc = 0) */

#define MAG_STATUS_ERROR_BIT (1u << 4) /* Status byte bit 4, datasheet 15.2 */

#define MAG_I2C_TIMEOUT_MS 50u

/* Filter/oversampling register and the two fields configured below.
 *
 * The datasheet's register map (Figure 23) survives OCR badly, so for the
 * record, the reconstruction this relies on -- corroborated by the bit
 * widths adding up exactly and by published community drivers:
 *   bits [1:0]   OSR        bits [10:9]  RES_Z
 *   bits [4:2]   DIG_FILT   bits [12:11] OSR2
 *   bits [6:5]   RES_X      bits [8:7]   RES_Y (straddles the byte boundary)
 * Note DIG_FILT is 3 bits wide, not the 2 that Table 16 claims -- Tables 19
 * and 20 index it 0..7, so the datasheet contradicts itself there.
 *
 * Only bits [4:0] are ever written (read-modify-write), so even if the
 * upper half of that reconstruction were wrong, RES_XYZ/OSR2 are preserved
 * and the sensitivity scaling used in Magnetometer_ReadMeasurement()
 * cannot be silently invalidated. */
#define MAG_REG_FILTER 0x02u
#define MAG_REG_FILTER_OSR_MASK 0x0003u      /* bits [1:0] */
#define MAG_REG_FILTER_DIG_FILT_MASK 0x001Cu /* bits [4:2] */

/* DIG_FILT = 5, OSR = 1 -> ~13.75 ms per T+X+Y+Z conversion, ~66 Hz
 * ceiling, ~0.8 uT of noise (datasheet Tables 19/20 and Figure 2).
 *
 * The power-on defaults (both 0) are NOT a legal combination with the
 * default HALLCONF=0xC -- that cell is marked red in Tables 19/20, and
 * section 16.2 says as much outright -- besides being the noisiest
 * setting available (~5 uT, i.e. 10% of the Earth's field). */
#define MAG_OSR 1u
#define MAG_DIG_FILT 5u

/* Bound on TCONV_SMM (datasheet Tables 8/12/19) for the settings above:
 * 13.75 ms of conversion plus TSTBY + TACTIVE + TCONV_END (~0.7 ms), and
 * then margin for the oscillator's +-5% trim and +-5% thermal drift.
 * Only used by the blocking Magnetometer_Read(); the interrupt-driven
 * path waits on the sensor's INT pin instead. */
#define MAG_CONVERSION_DELAY_MS 20u

/* Sensitivity at the sensor's power-on defaults (GAIN_SEL=0x7, HALLCONF=0xC,
 * RES_XYZ=0x00, TCMP_EN=0 -- see header comment and datasheet Table 17,
 * T=25 degC column), already folded into Tesla (Table 17 is in uT/LSB). */
#define MAG_SENS_XY_T_PER_LSB (0.150e-6f)
#define MAG_SENS_Z_T_PER_LSB (0.242e-6f)

/* Temperature sensor characterization (datasheet Table 7). */
#define MAG_TEMP_LSB_AT_25C 46244
#define MAG_TEMP_LSB_PER_DEGC 45.2f

/* === Private helpers ====================================================== */

/* One-byte command whose only reply is the status byte (SM, EX, ...). The
 * status is kept in mag->lastStatus -- it is the sensor's only way of
 * saying whether it accepted a command -- and the command fails if the
 * sensor flags an error. */
static bool Magnetometer_SimpleCommand(Magnetometer_t *mag, uint8_t cmd) {
  uint16_t addr = (uint16_t)(mag->i2cAddress << 1);

  if (HAL_I2C_Master_Transmit(mag->hi2c, addr, &cmd, 1, MAG_I2C_TIMEOUT_MS) !=
      HAL_OK) {
    return false;
  }

  uint8_t status = 0;
  if (HAL_I2C_Master_Receive(mag->hi2c, addr, &status, 1,
                              MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  mag->lastStatus = status;

  return (status & MAG_STATUS_ERROR_BIT) == 0;
}

/* Read Register (datasheet Figure 19): the register address goes out
 * shifted left by two bits, and the reply is a status byte followed by the
 * 16-bit value, MSB first. */
static bool Magnetometer_ReadRegister(Magnetometer_t *mag, uint8_t reg,
                                       uint16_t *value) {
  uint16_t addr = (uint16_t)(mag->i2cAddress << 1);
  uint8_t cmd[2] = {MAG_CMD_RR, (uint8_t)(reg << 2)};

  if (HAL_I2C_Master_Transmit(mag->hi2c, addr, cmd, sizeof(cmd),
                               MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  uint8_t rx[3]; /* status, D15..D8, D7..D0 */
  if (HAL_I2C_Master_Receive(mag->hi2c, addr, rx, sizeof(rx),
                              MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  mag->lastStatus = rx[0];
  if ((rx[0] & MAG_STATUS_ERROR_BIT) != 0) {
    return false;
  }

  *value = (uint16_t)((rx[1] << 8) | rx[2]);
  return true;
}

/* Write Register (datasheet Figure 20): command, then the 16-bit value MSB
 * first, then the register address shifted left by two bits. Writes land in
 * volatile memory only, so this has to be redone on every power-up -- which
 * is what Magnetometer_Init() is for. */
static bool Magnetometer_WriteRegister(Magnetometer_t *mag, uint8_t reg,
                                        uint16_t value) {
  uint16_t addr = (uint16_t)(mag->i2cAddress << 1);
  uint8_t cmd[4] = {MAG_CMD_WR, (uint8_t)(value >> 8), (uint8_t)(value & 0xFFu),
                    (uint8_t)(reg << 2)};

  if (HAL_I2C_Master_Transmit(mag->hi2c, addr, cmd, sizeof(cmd),
                               MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  uint8_t status = 0;
  if (HAL_I2C_Master_Receive(mag->hi2c, addr, &status, 1,
                              MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  mag->lastStatus = status;
  return (status & MAG_STATUS_ERROR_BIT) == 0;
}

/* === API =================================================================== */

void Magnetometer_Config(Magnetometer_t *mag, I2C_HandleTypeDef *hi2c,
                          uint8_t i2cAddress) {
  mag->hi2c = hi2c;
  mag->i2cAddress = i2cAddress;
}

bool Magnetometer_Init(Magnetometer_t *mag) {
  mag->initStep = Magnetometer_InitNotStarted;
  HAL_Delay(2); /* TPOR: power-on-reset completion time, max 1.5 ms */

  uint16_t addr = (uint16_t)(mag->i2cAddress << 1);
  if (HAL_I2C_IsDeviceReady(mag->hi2c, addr, 2, MAG_I2C_TIMEOUT_MS) !=
      HAL_OK) {
    return false;
  }
  mag->initStep = Magnetometer_InitProbed;

  /* Exit Mode first: harmless if the sensor was already idle, and it gets it
   * out of any mode left over from before an MCU reset (the sensor keeps
   * running across one). */
  if (!Magnetometer_Exit(mag)) {
    return false;
  }
  mag->initStep = Magnetometer_InitIdle;

  /* Set the oversampling/filtering, leaving every other field in the
   * register untouched (read-modify-write). Must happen while the sensor
   * is idle -- register writes are rejected during burst mode -- which the
   * Exit Mode command above guarantees. */
  uint16_t filterReg = 0;
  if (!Magnetometer_ReadRegister(mag, MAG_REG_FILTER, &filterReg)) {
    return false;
  }
  mag->filterRegRead = filterReg;
  mag->initStep = Magnetometer_InitFilterRead;

  filterReg &= (uint16_t)~(MAG_REG_FILTER_OSR_MASK |
                            MAG_REG_FILTER_DIG_FILT_MASK);
  filterReg |= (uint16_t)(MAG_OSR & 0x3u);
  filterReg |= (uint16_t)((MAG_DIG_FILT & 0x7u) << 2);
  mag->filterRegWritten = filterReg;

  if (!Magnetometer_WriteRegister(mag, MAG_REG_FILTER, filterReg)) {
    return false;
  }
  mag->initStep = Magnetometer_InitDone;

  return true;
}

bool Magnetometer_Read(Magnetometer_t *mag, Magnetometer_Sample_t *sample) {
  if (!Magnetometer_StartMeasurement(mag)) {
    return false;
  }
  HAL_Delay(MAG_CONVERSION_DELAY_MS);
  return Magnetometer_ReadMeasurement(mag, sample);
}

bool Magnetometer_StartMeasurement(Magnetometer_t *mag) {
  return Magnetometer_SimpleCommand(mag, MAG_CMD_SM_TXYZ);
}

bool Magnetometer_Exit(Magnetometer_t *mag) {
  return Magnetometer_SimpleCommand(mag, MAG_CMD_EXIT);
}

bool Magnetometer_ReadMeasurement(Magnetometer_t *mag,
                                   Magnetometer_Sample_t *sample) {
  uint16_t addr = (uint16_t)(mag->i2cAddress << 1);

  uint8_t cmd = MAG_CMD_RM_TXYZ;
  if (HAL_I2C_Master_Transmit(mag->hi2c, addr, &cmd, 1, MAG_I2C_TIMEOUT_MS) !=
      HAL_OK) {
    return false;
  }

  /* Status byte, then T, X, Y, Z as big-endian int16 pairs (datasheet
   * Figure 17, "RM command-XYZT"). */
  uint8_t rx[9];
  if (HAL_I2C_Master_Receive(mag->hi2c, addr, rx, sizeof(rx),
                              MAG_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  mag->lastStatus = rx[0];
  if ((rx[0] & MAG_STATUS_ERROR_BIT) != 0) {
    return false;
  }

  uint16_t rawT = (uint16_t)((rx[1] << 8) | rx[2]);
  int16_t rawX = (int16_t)((rx[3] << 8) | rx[4]);
  int16_t rawY = (int16_t)((rx[5] << 8) | rx[6]);
  int16_t rawZ = (int16_t)((rx[7] << 8) | rx[8]);

  sample->fieldX_T = (float)rawX * MAG_SENS_XY_T_PER_LSB;
  sample->fieldY_T = (float)rawY * MAG_SENS_XY_T_PER_LSB;
  sample->fieldZ_T = (float)rawZ * MAG_SENS_Z_T_PER_LSB;
  sample->temperature_degC =
      25.0f + ((float)rawT - (float)MAG_TEMP_LSB_AT_25C) /
                  MAG_TEMP_LSB_PER_DEGC;

  return true;
}
