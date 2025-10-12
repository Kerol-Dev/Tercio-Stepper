// -----------------------------------------------------------------------------
// CanCmdBus.cpp
// RX payload: [CMD][PAY...], TX payload: [CMD][PAY...]
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ACANFD_STM32_from_cpp.h>
#include <ACANFD_STM32_Settings.h>
#include <ACANFD_STM32.h>
#include <cstring>
#include "CanCmdBus.h"

// -----------------------------------------------------------------------------
// Defaults
// -----------------------------------------------------------------------------
static constexpr uint32_t CAN_CTRL_ID_MASK_DEFAULT = 0x000;
static constexpr uint32_t CAN_CTRL_ID_FILT_DEFAULT = 0x000;

#ifndef CANCMD_DEFAULT_RX_PIN
  #define CANCMD_DEFAULT_RX_PIN PA11
#endif
#ifndef CANCMD_DEFAULT_TX_PIN
  #define CANCMD_DEFAULT_TX_PIN PA12
#endif

// -----------------------------------------------------------------------------
// Internals
// -----------------------------------------------------------------------------
namespace CanCmdBus {

struct Entry { uint8_t cmd; Handler fn; };
static Entry   s_table[CANCMD_MAX_HANDLERS];
static uint8_t s_count = 0;

static uint16_t s_idFilter    = static_cast<uint16_t>(CAN_CTRL_ID_FILT_DEFAULT);
static uint16_t s_idMask      = static_cast<uint16_t>(CAN_CTRL_ID_MASK_DEFAULT);
static uint16_t s_defaultTxId = 0x000;

// Map to valid CAN-FD lengths (0..8,12,16,20,24,32,48,64)
static uint8_t normalizeFdLen(uint8_t n) {
  static constexpr uint8_t steps[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  const uint8_t count = static_cast<uint8_t>(sizeof(steps) / sizeof(steps[0]));
  for (uint8_t i = 0; i < count; ++i) {
    if (n <= steps[i]) return steps[i];
  }
  return 64;
}

// Find handler index; returns existing index or 0xFF
static uint8_t findIndex(uint8_t cmd) {
  for (uint8_t i = 0; i < s_count; ++i) {
    if (s_table[i].cmd == cmd) return i;
  }
  return 0xFF;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool begin(uint32_t nominal_bps,
           uint8_t  data_factor,
           uint8_t  rxPin,
           uint8_t  txPin,
           bool     normalFD)
{
  ACANFD_STM32_Settings settings(nominal_bps, (DataBitRateFactor)data_factor);
  settings.mModuleMode = normalFD
                       ? ACANFD_STM32_Settings::NORMAL_FD
                       : ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK;

  settings.mRxPin = (rxPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_RX_PIN : rxPin;
  settings.mTxPin = (txPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_TX_PIN : txPin;

  const uint32_t err = fdcan1.beginFD(settings);
  if (err != 0) {
    return false;
  }

  s_count = 0;
  for (uint8_t i = 0; i < CANCMD_MAX_HANDLERS; ++i) {
    s_table[i].cmd = 0;
    s_table[i].fn  = nullptr;
  }
  return true;
}

void poll() {
  CANFDMessage m;
  while (fdcan1.receiveFD0(m)) {
    const uint16_t id = static_cast<uint16_t>(m.id & 0x7FF);
    if ((id & s_idMask) != (s_idFilter & s_idMask)) continue;

    if (m.len == 0) continue;
    const uint8_t  cmd     = m.data[0];
    const uint8_t* payload = &m.data[1];
    const uint8_t  payLen  = (m.len > 1) ? static_cast<uint8_t>(m.len - 1) : 0;

    const uint8_t idx = findIndex(cmd);
    if (idx == 0xFF || s_table[idx].fn == nullptr) continue;

    CmdFrame cf;
    cf.id      = id;
    cf.cmd     = cmd;
    cf.payload = payload;
    cf.len     = payLen;

    s_table[idx].fn(cf);
  }
}

bool registerHandler(uint8_t cmd, Handler fn) {
  const uint8_t idx = findIndex(cmd);
  if (idx != 0xFF) { s_table[idx].fn = fn; return true; }

  if (s_count >= CANCMD_MAX_HANDLERS) return false;
  s_table[s_count].cmd = cmd;
  s_table[s_count].fn  = fn;
  ++s_count;
  return true;
}

void setIdFilter(uint16_t filter, uint16_t mask) {
  s_idFilter = static_cast<uint16_t>(filter & 0x7FF);
  s_idMask   = static_cast<uint16_t>(mask   & 0x7FF);
}

void setDefaultTxId(uint16_t id) {
  s_defaultTxId = static_cast<uint16_t>(id & 0x7FF);
}

// -----------------------------------------------------------------------------
// TX helpers
// -----------------------------------------------------------------------------
bool send(uint16_t id, uint8_t cmd, const void* data, uint8_t len) {
  uint8_t buf[64] = {0};
  buf[0] = cmd;

  if (len > 0 && data != nullptr) {
    const uint8_t maxPay = 63;
    if (len > maxPay) len = maxPay;
    memcpy(&buf[1], data, len);
  }

  const uint8_t rawLen = static_cast<uint8_t>(1 + len);
  CANFDMessage m;
  m.id  = static_cast<uint32_t>(id & 0x7FF);
  m.len = normalizeFdLen(rawLen);
  memset(m.data, 0, sizeof(m.data));
  memcpy(m.data, buf, rawLen);

  const uint32_t st = fdcan1.tryToSendReturnStatusFD(m);
  return (st == 0);
}

bool sendDefault(uint8_t cmd, const void* data, uint8_t len) {
  return send(s_defaultTxId, cmd, data, len);
}

bool sendF32(uint16_t id, uint8_t cmd, float value) {
  return send(id, cmd, &value, sizeof(value));
}
bool sendF32(uint8_t cmd, float value) {
  return send(s_defaultTxId, cmd, &value, sizeof(value));
}

bool sendI32(uint16_t id, uint8_t cmd, int32_t value) {
  return send(id, cmd, &value, sizeof(value));
}
bool sendI32(uint8_t cmd, int32_t value) {
  return send(s_defaultTxId, cmd, &value, sizeof(value));
}

bool sendU16(uint16_t id, uint8_t cmd, uint16_t value) {
  return send(id, cmd, &value, sizeof(value));
}
bool sendU16(uint8_t cmd, uint16_t value) {
  return send(s_defaultTxId, cmd, &value, sizeof(value));
}

bool sendU8(uint16_t id, uint8_t cmd, uint8_t value) {
  return send(id, cmd, &value, sizeof(value));
}
bool sendU8(uint8_t cmd, uint8_t value) {
  return send(s_defaultTxId, cmd, &value, sizeof(value));
}

bool sendStruct(uint16_t id, uint8_t cmd, const void* p, size_t n) {
  if (p == nullptr) return false;
  if (n > 63) n = 63;
  return send(id, cmd, p, static_cast<uint8_t>(n));
}

} // namespace CanCmdBus