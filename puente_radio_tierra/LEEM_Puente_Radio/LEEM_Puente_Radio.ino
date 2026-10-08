/*
  Puente transparente entre el puerto serie y la radio LoRa
  Placa: DX-PJ26 v1.2 (STM32F103C8T6, "Blue Pill") con modulo DX-LR30 (SX1262)
  Libreria: RadioLib 7.7.1

  Estacion de tierra. Hace de modem en los dos sentidos:

    radio -> serie   lo que llega por el aire sale por el puerto serie, CON LA
                     CALIDAD DEL ENLACE anadida al final (RSSI, SNR y
                     desplazamiento de frecuencia).
    serie -> radio   lo que llega por el puerto serie se pone en el aire tal
                     cual, sin interpretarlo.

  No entiende el contenido: el paquete de vuelo de 24 bytes, un comando o
  cualquier otra cosa pasan igual. Quien decodifica es el PC.

  POR QUE VA ENMARCADO. El puerto serie es un flujo de bytes y la radio son
  paquetes. Sin una longitud explicita no hay forma de saber donde acaba uno y
  empieza el siguiente, y un solo byte perdido desincronizaria el flujo PARA
  SIEMPRE, sin manera de recuperarse. Con los dos bytes de sincronismo y la
  longitud, el receptor se reengancha solo en el siguiente paquete.

  TRAMA radio -> serie
      AA 55        sincronismo
      len          1 byte, longitud del payload
      payload      len bytes, tal cual llegaron por el aire
      rssi         int8    dBm
      snr          int8    en pasos de 0,25 dB  (+/-31,75 dB)
      df           int16   Hz, little endian, con signo
      flags        ver abajo
      crc8         sobre len, payload y metadatos

  TRAMA serie -> radio
      AA 55  len  payload[len]  crc8
  len = 0 se descarta: la longitud cero esta reservada para el acuse.

  FLAGS
      bit 0   el CRC del paquete LoRa venia mal
      bit 1   ACUSE LOCAL: la trama no viene del aire, la genera esta placa
      bit 2   solo con el bit 1: la transmision fallo

  ACUSE DE TRANSMISION. Despues de cada envio al aire la placa devuelve por
  serie una trama con len = 0 y el bit 1 puesto, diciendo si salio o no. Hace
  falta porque el enlace de subida es el lado debil y, sin esto, el operador
  manda un comando y no tiene forma de saber si llego siquiera a radiarse. Un
  decodificador que solo quiera telemetria lo ignora solo: len = 0, sin
  payload. Son 9 bytes y los comandos son escasos.

  LOS PAQUETES CON CRC MALO TAMBIEN SE REENVIAN, marcados en flags. En una
  prueba de alcance eso es justo lo interesante: saber que algo llego pero
  corrupto distingue "fuera de cobertura" de "no transmitio nadie", y es la
  senal que marca el borde del alcance.

  CICLO DE TRABAJO. Todo lo que el PC mande por serie sale al aire sin filtro.
  La banda de 869,4 a 869,65 MHz permite el 10 % medido sobre una hora, o sea
  360 s de emision por hora. Un paquete de 24 bytes ocupa 30,9 ms: son unos
  11 600 paquetes por hora. Este firmware NO lo vigila; que no se dispare
  depende de quien escriba el lado del PC.

  MIENTRAS TRANSMITE ESTA SORDO. La radio no puede recibir y transmitir a la
  vez, asi que durante los ~31 ms de una transmision se pierde lo que llegue.
  Con el trafico previsto no deberia notarse, pero conviene saberlo.

  Configuracion de radio IDENTICA a la de la placa del cohete
  (bsp/sx1262/bsp_sx1262.c). Si cambia un solo parametro, no se oyen.

  Placa en el IDE: Herramientas > Placa > STM32 boards > Generic STM32F1 series
  Board part number: BluePill F103C8
  Serie: USART1 (PA9 = TX, PA10 = RX) a 115200, con un conversor USB-serie.
*/

#include <RadioLib.h>

// ---- Pines de la DX-PJ26 v1.2 ----
#define PIN_NSS    PA4
#define PIN_DIO1   PC15
#define PIN_RESET  PA3
#define PIN_BUSY   PA2
#define PIN_RXEN   PA1
#define PIN_TXEN   PA0
// SCK=PA5, MISO=PA6, MOSI=PA7: SPI1 por hardware.

// ---- Radio: los mismos parametros que el cohete ----
#define FREQ_BASE_MHZ   869.525
#define BW_KHZ          250.0
#define SPREADING       7
#define CODING_RATE     5
#define PREAMBLE_SYM    8
#define TX_POWER_DBM    22

/*
  Correccion de frecuencia de esta placa. Los condensadores del cristal del
  modulo no son ajustables (se probo: escribir XTA/XTB no mueve la frecuencia),
  asi que la desviacion se compensa moviendo la portadora. -2000 Hz deja el
  error por debajo de 3,4 kHz medido desde los dos extremos.
*/
#define FREQ_OFFSET_HZ  (-2000)

// ---- Trama ----
#define SYNC0 0xAA
#define SYNC1 0x55
#define PAYLOAD_MAX 255
#define META_BYTES  5    // rssi, snr, df (2), flags

#define FLAG_CRC_ERROR 0x01
#define FLAG_LOCAL     0x02
#define FLAG_TX_FAIL   0x04

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RESET, PIN_BUSY);

volatile bool radioFlag = false;
void setFlag(void) { radioFlag = true; }

/*
  CRC-8 (polinomio 0x07). No pretende proteger contra nada sofisticado: solo
  detecta que la trama de serie ha llegado entera y alineada.
*/
static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

/* ---- radio -> serie ---------------------------------------------------- */

static void enviarASerie(const uint8_t *payload, uint8_t len, float rssi,
                         float snr, float df, uint8_t flags) {
  uint8_t cuerpo[PAYLOAD_MAX + META_BYTES];

  if (len > 0) {
    memcpy(cuerpo, payload, len);
  }

  // SNR en pasos de 0,25 dB; se acota para que quepa en int8
  long snrRaw = lround(snr * 4.0);
  if (snrRaw > 127) snrRaw = 127;
  if (snrRaw < -128) snrRaw = -128;

  long rssiRaw = lround(rssi);
  if (rssiRaw > 127) rssiRaw = 127;
  if (rssiRaw < -128) rssiRaw = -128;

  long dfRaw = lround(df);
  if (dfRaw > 32767) dfRaw = 32767;
  if (dfRaw < -32768) dfRaw = -32768;

  cuerpo[len + 0] = (uint8_t)(int8_t)rssiRaw;
  cuerpo[len + 1] = (uint8_t)(int8_t)snrRaw;
  cuerpo[len + 2] = (uint8_t)((uint16_t)(int16_t)dfRaw & 0xFF);
  cuerpo[len + 3] = (uint8_t)(((uint16_t)(int16_t)dfRaw >> 8) & 0xFF);
  cuerpo[len + 4] = flags;

  const uint8_t total = (uint8_t)(len + META_BYTES);

  // El CRC cubre la longitud, el payload y los metadatos
  uint8_t cabecera[1] = {len};
  uint8_t crc = crc8(cabecera, 1);
  for (uint8_t i = 0; i < total; i++) {
    uint8_t b = cuerpo[i];
    crc ^= b;
    for (uint8_t k = 0; k < 8; k++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }

  Serial.write(SYNC0);
  Serial.write(SYNC1);
  Serial.write(len);
  Serial.write(cuerpo, total);
  Serial.write(crc);
}

/* ---- serie -> radio ---------------------------------------------------- */

/*
  Maquina de estados que va reconstruyendo la trama byte a byte segun llega.
  Se hace asi y no leyendo bloques porque el PC puede partir la trama en
  varios envios: el puerto serie no garantiza que llegue de una pieza.
*/
enum EstadoRx { ESPERA_SYNC0, ESPERA_SYNC1, ESPERA_LEN, ESPERA_PAYLOAD, ESPERA_CRC };
EstadoRx estado = ESPERA_SYNC0;
uint8_t bufSerie[PAYLOAD_MAX];
uint8_t serieLen = 0;
uint8_t serieIdx = 0;

static void procesarSerie(void) {
  while (Serial.available() > 0) {
    const uint8_t b = (uint8_t)Serial.read();

    switch (estado) {
      case ESPERA_SYNC0:
        if (b == SYNC0) estado = ESPERA_SYNC1;
        break;

      case ESPERA_SYNC1:
        // Si no es el segundo byte de sincronismo puede que este sea el
        // primero de una trama nueva: no se pierde la oportunidad.
        estado = (b == SYNC1) ? ESPERA_LEN
                              : ((b == SYNC0) ? ESPERA_SYNC1 : ESPERA_SYNC0);
        break;

      case ESPERA_LEN:
        serieLen = b;
        serieIdx = 0;
        if (serieLen == 0) {
          // len = 0 esta reservado para el acuse local, no se radia
          estado = ESPERA_SYNC0;
        } else {
          estado = ESPERA_PAYLOAD;
        }
        break;

      case ESPERA_PAYLOAD:
        bufSerie[serieIdx++] = b;
        if (serieIdx >= serieLen) estado = ESPERA_CRC;
        break;

      case ESPERA_CRC: {
        uint8_t comprobar[PAYLOAD_MAX + 1];
        comprobar[0] = serieLen;
        memcpy(&comprobar[1], bufSerie, serieLen);

        if (crc8(comprobar, (size_t)serieLen + 1) == b) {
          const int estadoTx = radio.transmit(bufSerie, serieLen);

          // transmit() deja el chip en standby: hay que volver a escuchar
          radioFlag = false;   // la transmision tambien levanta DIO1
          radio.startReceive();

          const uint8_t flags =
              (uint8_t)(FLAG_LOCAL |
                        ((estadoTx == RADIOLIB_ERR_NONE) ? 0U : FLAG_TX_FAIL));
          enviarASerie(NULL, 0, 0.0f, 0.0f, 0.0f, flags);
        }
        // Una trama con el CRC mal se descarta sin avisar: al no llegar el
        // acuse, el PC ya sabe que no salio.
        estado = ESPERA_SYNC0;
        break;
      }
    }
  }
}

/* ---- Arranque ---------------------------------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(500);

  // Texto, no trama: un parser que busque AA 55 lo ignora sin enterarse
  Serial.println();
  Serial.println(F("# LEEM puente radio <-> serie"));
  Serial.println(F("# radio->serie: AA 55 len payload rssi snr df_lo df_hi flags crc8"));
  Serial.println(F("# serie->radio: AA 55 len payload crc8"));
  Serial.println(F("# flags: b0 crc malo  b1 acuse local  b2 fallo al transmitir"));

  radio.setRfSwitchPins(PIN_RXEN, PIN_TXEN);

  int state = radio.begin(FREQ_BASE_MHZ, BW_KHZ, SPREADING, CODING_RATE,
                          RADIOLIB_SX126X_SYNC_WORD_PUBLIC, TX_POWER_DBM,
                          PREAMBLE_SYM, 0.0, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("# ERROR al inicializar la radio, codigo "));
    Serial.println(state);
    while (true) { delay(1000); }
  }

  radio.explicitHeader();
  radio.setCRC(true);
  radio.invertIQ(false);
  radio.forceLDRO(false);
  radio.setDio1Action(setFlag);
  radio.setFrequency(FREQ_BASE_MHZ + (FREQ_OFFSET_HZ / 1000000.0), false);

  radio.startReceive();
  Serial.println(F("# escuchando"));
}

void loop() {
  // ---- radio -> serie ----
  if (radioFlag) {
    radioFlag = false;

    size_t len = radio.getPacketLength();
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;

    uint8_t payload[PAYLOAD_MAX];
    const int state = radio.readData(payload, len);

    if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH)) {
      const uint8_t flags =
          (state == RADIOLIB_ERR_CRC_MISMATCH) ? FLAG_CRC_ERROR : 0U;
      enviarASerie(payload, (uint8_t)len, radio.getRSSI(), radio.getSNR(),
                   radio.getFrequencyError(), flags);
    }

    radio.startReceive();
  }

  // ---- serie -> radio ----
  procesarSerie();
}
