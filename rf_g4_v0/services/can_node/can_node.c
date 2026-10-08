/**
 * @file    can_node.c
 * @brief   This board as a node of the common CAN1 bus between boards.
 */
#include "can_node.h"

#include <string.h>

#include "bsp_can1.h"
#include "can_protocol/can_ids.h"
#include "can_protocol/can_pack.h"
#include "can_protocol/can_protocol_version.h"

/* One filter for two node fields: broadcast (0000) and rf (0100) only differ
 * in bit 6 of the ID, so leaving that bit out of the mask accepts
 * 0x100-0x10F and 0x140-0x14F and nothing else (ICD 5.3). */
#define CAN_NODE_FILTER_ID CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_BROADCAST, 0x0U)
#define CAN_NODE_FILTER_MASK \
  ((CAN_ID_CLASS_MASK | CAN_ID_NODE_MASK) & ~CAN_ID(0x0U, CAN_NODE_RF, 0x0U))

/* Only touched from the main loop. */
static bool s_awake = false;
static bool s_wake_up_pending = false;

static uint8_t s_radio_payload[CAN_NODE_RADIO_PAYLOAD_MAX];
static uint8_t s_radio_len = 0;
static bool s_radio_pending = false;

static uint32_t s_wake_up_count = 0;
static uint32_t s_incompatible_count = 0;
static uint32_t s_tx_error_count = 0;
static uint32_t s_radio_count = 0;
static uint32_t s_radio_dropped_count = 0;
static uint32_t s_rx_forwarded_count = 0;
static uint32_t s_rx_dropped_count = 0;
static uint32_t s_bus_off_count = 0;

static void on_wake_up(const bsp_can1_frame_t *frame) {
  if (frame->len < 2U) {
    return;
  }

  if (!CAN_PROTOCOL_IS_COMPATIBLE(can_get_u16_le(&frame->data[0]))) {
    s_incompatible_count++;
    return;
  }

  s_wake_up_count++;
  s_awake = true;
  s_wake_up_pending = true;

  if (!bsp_can1_send(CAN_ID_WAKE_UP_ACK(CAN_NODE_RF), NULL, 0U)) {
    s_tx_error_count++;
  }
}

/* ICD 6.10: the whole payload goes on the air, untouched. */
static void on_rf_transmit(const bsp_can1_frame_t *frame) {
  if (!s_awake || frame->len == 0U ||
      frame->len > CAN_NODE_RADIO_PAYLOAD_MAX) {
    s_radio_dropped_count++;
    return;
  }

  if (s_radio_pending) {
    /* The previous one has not been transmitted yet: the newest wins */
    s_radio_dropped_count++;
  }

  memcpy(s_radio_payload, frame->data, frame->len);
  s_radio_len = (uint8_t)frame->len;
  s_radio_pending = true;
  s_radio_count++;
}

bool can_node_init(void) {
  return bsp_can1_start(CAN_NODE_FILTER_ID, CAN_NODE_FILTER_MASK);
}

void can_node_poll(void) {
  bsp_can1_frame_t frame;

  /* A bus-off never clears by itself (bsp_can1.h). Here, so that it is
   * covered both while waiting for the wake up and in the main loop. */
  if (bsp_can1_recover_bus_off()) {
    s_bus_off_count++;
  }

  while (bsp_can1_poll_rx(&frame)) {
    switch (frame.id) {
    case CAN_ID_CMD_WAKE_UP:
      on_wake_up(&frame);
      break;

    case CAN_ID_CMD_RF_TRANSMIT:
      on_rf_transmit(&frame);
      break;

    default:
      /* Other commands through the same filter: not handled yet */
      break;
    }
  }
}

bool can_node_poll_wake_up(void) {
  can_node_poll();

  bool pending = s_wake_up_pending;
  s_wake_up_pending = false;
  return pending;
}

bool can_node_is_awake(void) { return s_awake; }

bool can_node_take_radio_payload(uint8_t *buf, uint8_t buf_size, uint8_t *len) {
  if (!s_radio_pending || buf == NULL || len == NULL) {
    return false;
  }
  s_radio_pending = false;

  if (s_radio_len > buf_size) {
    s_radio_dropped_count++;
    return false;
  }

  memcpy(buf, s_radio_payload, s_radio_len);
  *len = s_radio_len;
  return true;
}

/* ICD 6.12: what the radio heard goes up to the core untouched, behind a
 * header with the link quality (see can_node.h). */
bool can_node_send_received(const uint8_t *payload, uint8_t length,
                            int8_t rssi_dbm, int8_t snr_db,
                            int32_t freq_error_hz) {
  if (!s_awake || payload == NULL || length == 0U ||
      length > CAN_NODE_RX_PAYLOAD_MAX) {
    s_rx_dropped_count++;
    return false;
  }
  int32_t df = freq_error_hz;
  if (df > 32767) {
    df = 32767;
  } else if (df < -32768) {
    df = -32768;
  }
  uint8_t frame[CAN_NODE_RADIO_PAYLOAD_MAX];
  frame[0] = length;
  frame[1] = (uint8_t)rssi_dbm;
  frame[2] = (uint8_t)snr_db;
  can_put_i16_le(&frame[3], (int16_t)df);
  memcpy(&frame[CAN_NODE_RX_HEADER_BYTES], payload, length);
  if (!bsp_can1_send(CAN_ID_RF_RECEIVED, frame,
                     (uint8_t)(CAN_NODE_RX_HEADER_BYTES + length))) {
    s_tx_error_count++;
    s_rx_dropped_count++;
    return false;
  }
  s_rx_forwarded_count++;
  return true;
}

uint32_t can_node_rx_forwarded_count(void) { return s_rx_forwarded_count; }

uint32_t can_node_rx_dropped_count(void) { return s_rx_dropped_count; }

uint32_t can_node_bus_off_count(void) { return s_bus_off_count; }

uint32_t can_node_wake_up_count(void) { return s_wake_up_count; }

uint32_t can_node_incompatible_count(void) { return s_incompatible_count; }

uint32_t can_node_tx_error_count(void) { return s_tx_error_count; }

uint32_t can_node_radio_count(void) { return s_radio_count; }

uint32_t can_node_radio_dropped_count(void) { return s_radio_dropped_count; }
