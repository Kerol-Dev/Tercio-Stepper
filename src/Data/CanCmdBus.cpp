// -----------------------------------------------------------------------------
// CanCmdBus.cpp
// Modular CAN-FD command bus for STM32 (Arduino/PlatformIO)
// RX: Matches frames from your bridge: CAN data = [CMD][PAY...]
// TX: Provides helpers to send [CMD][PAY...] frames (float/int/struct/raw)
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ACANFD_STM32_from_cpp.h>
#include <ACANFD_STM32_Settings.h>
#include <ACANFD_STM32.h>
#include "CanCmdBus.h"

// ---------------- Config defaults ----------------
static constexpr uint32_t CAN_CTRL_ID_MASK_DEFAULT = 0x000; // accept all
static constexpr uint32_t CAN_CTRL_ID_FILT_DEFAULT = 0x000;

// Default pins for FDCAN1 on STM32G431 (AF9)
#ifndef CANCMD_DEFAULT_RX_PIN
#define CANCMD_DEFAULT_RX_PIN PA11
#endif
#ifndef CANCMD_DEFAULT_TX_PIN
#define CANCMD_DEFAULT_TX_PIN PA12
#endif

// ---------------- Internals ----------------
namespace CanCmdBus {

static Print*       s_dbg      = nullptr;

struct Entry { uint8_t cmd; Handler fn; };
static Entry   s_table[CANCMD_MAX_HANDLERS];
static uint8_t s_count   = 0;

static uint16_t s_idFilter = (uint16_t)CAN_CTRL_ID_FILT_DEFAULT;
static uint16_t s_idMask   = (uint16_t)CAN_CTRL_ID_MASK_DEFAULT;

static uint16_t s_defaultTxId = 0x000; // default TX ID (can be changed)

static inline void logln(const char* s)   { if (s_dbg) s_dbg->println(s); }
static inline void log  (const char* s)   { if (s_dbg) s_dbg->print(s);   }
static inline void logHex(uint32_t v)     { if (s_dbg) s_dbg->println(v, HEX); }
static inline void logCmd(uint8_t c)      { if (s_dbg) { s_dbg->print("CMD 0x"); s_dbg->println(c, HEX); } }

// Find index of cmd in table; returns existing index or 0xFF
static uint8_t findIndex(uint8_t cmd) {
  for (uint8_t i = 0; i < s_count; ++i) if (s_table[i].cmd == cmd) return i;
  return 0xFF;
}

// Map arbitrary length to CAN-FD DLC step (we clamp to 0..64; your bridge uses <=32)
static uint8_t lenToDLC(uint8_t n) {
  static const uint8_t steps[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  for (uint8_t i = 0; i < sizeof(steps); ++i) {
    if (n <= steps[i]) return steps[i];
  }
  return 64;
}

bool begin(uint32_t nominal_bps,
           uint8_t  data_factor,
           uint8_t  rxPin,
           uint8_t  txPin,
           bool     normalFD)
{
  // Configure timings
  ACANFD_STM32_Settings settings(nominal_bps, (DataBitRateFactor)data_factor);
  settings.mModuleMode = normalFD ? ACANFD_STM32_Settings::NORMAL_FD
                                  : ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK;

  settings.mRxPin = (rxPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_RX_PIN : rxPin;
  settings.mTxPin = (txPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_TX_PIN : txPin;

  const uint32_t err = fdcan1.beginFD(settings);
  if (err != 0) {
    if (s_dbg) { s_dbg->print("FDCAN init error: 0x"); s_dbg->println(err, HEX); }
    return false;
  }

  // Clear registry
  s_count = 0;
  for (uint8_t i = 0; i < CANCMD_MAX_HANDLERS; ++i) { s_table[i].cmd = 0; s_table[i].fn = nullptr; }

  return true;
}

void poll() {
  CANFDMessage m;
  // Drain RX FIFO quickly
  while (fdcan1.receiveFD0(m)) {
    // ID filter check (standard ID)
    const uint16_t id = (uint16_t)(m.id & 0x7FF);
    if ((id & s_idMask) != (s_idFilter & s_idMask)) {
      continue; // not for us
    }

    if (m.len == 0) continue; // must have at least CMD
    const uint8_t  cmd     = m.data[0];
    const uint8_t* payload = &m.data[1];
    const uint8_t  payLen  = (m.len > 1) ? (uint8_t)(m.len - 1) : 0;

    // Dispatch
    const uint8_t idx = findIndex(cmd);
    if (idx == 0xFF || s_table[idx].fn == nullptr) {
      if (s_dbg) { s_dbg->print("Unknown "); logCmd(cmd); }
      continue;
    }

    CmdFrame cf;
    cf.id      = id;
    cf.cmd     = cmd;
    cf.payload = payload;
    cf.len     = payLen;

    s_table[idx].fn(cf);
  }
}

bool registerHandler(uint8_t cmd, Handler fn) {
  // If exists, overwrite
  const uint8_t idx = findIndex(cmd);
  if (idx != 0xFF) { s_table[idx].fn = fn; return true; }

  // New entry
  if (s_count >= CANCMD_MAX_HANDLERS) {
    if (s_dbg) logln("Handler table full");
    return false;
  }
  s_table[s_count].cmd = cmd;
  s_table[s_count].fn  = fn;
  s_count++;
  return true;
}

void setIdFilter(uint16_t filter, uint16_t mask) {
  s_idFilter = (uint16_t)(filter & 0x7FF); // standard 11-bit
  s_idMask   = (uint16_t)(mask   & 0x7FF);
}

void setDebug(Print* dbg) { s_dbg = dbg; }

void setDefaultTxId(uint16_t id) { s_defaultTxId = (uint16_t)(id & 0x7FF); }

// ------------------------- TX helpers -------------------------
bool send(uint16_t id, uint8_t cmd, const void* data, uint8_t len) {
  // Build [CMD][PAY...] into CAN data
  const uint8_t* p = (const uint8_t*)data;
  uint8_t buf[64] = {0};        // zero so padding bytes are deterministic
  buf[0] = cmd;

  if (len > 0 && p != nullptr) {
    // Clamp to 63 bytes because we already used 1 byte for CMD (max FD DLC 64)
    if (len > 63) len = 63;
    memcpy(&buf[1], p, len);
  }

  CANFDMessage m;
  m.id  = (uint32_t)(id & 0x7FF);
  m.len = lenToDLC((uint8_t)(1 + len));
  memset(m.data, 0, sizeof(m.data));
  memcpy(m.data, buf, (size_t)(1 + len));

  const uint32_t st = fdcan1.tryToSendReturnStatusFD(m);
  if (st != 0) {
    if (s_dbg) { s_dbg->print("CAN TX error 0x"); s_dbg->println(st, HEX); }
    return false;
  }
  return true;
}

bool sendDefault(uint8_t cmd, const void* data, uint8_t len) {
  return send(s_defaultTxId, cmd, data, len);
}

bool sendF32(uint16_t id, uint8_t cmd, float value) {
  return send(id, cmd, &value, 4);
}
bool sendF32(uint8_t cmd, float value) {
  return send(s_defaultTxId, cmd, &value, 4);
}

bool sendI32(uint16_t id, uint8_t cmd, int32_t value) {
  return send(id, cmd, &value, 4);
}
bool sendI32(uint8_t cmd, int32_t value) {
  return send(s_defaultTxId, cmd, &value, 4);
}

} // namespace CanCmdBus