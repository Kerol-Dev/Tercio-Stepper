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
namespace CanCmdBus
{

  struct Entry
  {
    uint8_t cmd;
    Handler fn;
  };
  static Entry s_table[CANCMD_MAX_HANDLERS];
  static uint8_t s_count = 0;

  static uint16_t s_idFilter = static_cast<uint16_t>(CAN_CTRL_ID_FILT_DEFAULT);
  static uint16_t s_idMask = static_cast<uint16_t>(CAN_CTRL_ID_MASK_DEFAULT);
  static uint16_t s_defaultTxId = 0x000;

  // Map to valid CAN-FD lengths (0..8,12,16,20,24,32,48,64)
  static uint8_t normalizeFdLen(uint8_t n)
  {
    static constexpr uint8_t steps[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    const uint8_t count = static_cast<uint8_t>(sizeof(steps) / sizeof(steps[0]));
    for (uint8_t i = 0; i < count; ++i)
    {
      if (n <= steps[i])
        return steps[i];
    }
    return 64;
  }

  // Find handler index; returns existing index or 0xFF
  static uint8_t findIndex(uint8_t cmd)
  {
    for (uint8_t i = 0; i < s_count; ++i)
    {
      if (s_table[i].cmd == cmd)
        return i;
    }
    return 0xFF;
  }

  // -----------------------------------------------------------------------------
  // Public API
  // -----------------------------------------------------------------------------
  bool begin(uint32_t nominal_bps,
             uint8_t data_factor,
             uint8_t rxPin,
             uint8_t txPin,
             bool normalFD)
  {
    ACANFD_STM32_Settings settings(nominal_bps, (DataBitRateFactor)data_factor);
    settings.mModuleMode = normalFD
                               ? ACANFD_STM32_Settings::NORMAL_FD
                               : ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK;

    settings.mRxPin = (rxPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_RX_PIN : rxPin;
    settings.mTxPin = (txPin == 0xFF) ? (uint8_t)CANCMD_DEFAULT_TX_PIN : txPin;

    const uint32_t err = fdcan1.beginFD(settings);
    if (err != 0)
    {
      return false;
    }

    s_count = 0;
    for (uint8_t i = 0; i < CANCMD_MAX_HANDLERS; ++i)
    {
      s_table[i].cmd = 0;
      s_table[i].fn = nullptr;
    }
    return true;
  }

  static inline bool cmd_len_bounds(uint8_t cmd, uint8_t &minL, uint8_t &maxL)
  {
    switch (cmd)
    {
    case 0x01:
      minL = maxL = 4;
      return true; // TARGET_ANGLE (f32)
    case 0x02:
      minL = maxL = 2;
      return true; // SET_CURRENT_MA (u16)
    case 0x03:
      minL = maxL = 4;
      return true; // SET_SPEED_LIMIT (f32)
    case 0x04:
      minL = maxL = 12;
      return true; // SET_PID (3*f32)
    case 0x05:
      minL = maxL = 2;
      return true; // SET_ID (u16 & 0x7FF)
    case 0x06:
      minL = maxL = 2;
      return true; // SET_MICROSTEPS (u16)
    case 0x07:
      minL = maxL = 1;
      return true; // SET_STEALTHCHOP (u8)
    case 0x08:
      minL = maxL = 1;
      return true; // SET_EXT_MODE (u8)
    case 0x09:
      minL = maxL = 1;
      return true; // SET_UNITS (u8)
    case 0x0A:
      minL = maxL = 1;
      return true; // SET_ENC_INVERT (u8)
    case 0x0B:
      minL = maxL = 1;
      return true; // SET_ENABLED (u8)
    case 0x0C:
      minL = maxL = 2;
      return true; // SET_STEPS_PER_REV (u16)
    case 0x0D:
      minL = maxL = 0;
      return true; // DO_CALIBRATE ()
    case 0x0E:
      minL = maxL = 11;
      return true; // DO_HOMING (<B f B f B)
    case 0x0F:
      minL = maxL = 1;
      return true; // SET_ENDSTOP (u8)
    case 0x10:
      minL = maxL = 1;
      return true; // SET_EXT_ENCODER (u8)
    case 0x11:
      minL = maxL = 4;
      return true; // SET_ACCEL_LIMIT (f32)
    case 0x12:
      minL = maxL = 1;
      return true; // SET_DIR_INVERT (u8)
    default:
      return false; // unknown command -> drop
    }
  }

  void poll()
  {
    CANFDMessage m;
    while (fdcan1.receiveFD0(m))
    {
      const uint16_t id = static_cast<uint16_t>(m.id & 0x7FF);
      if ((id & s_idMask) != (s_idFilter & s_idMask))
        continue;

      // Basic sanity
      if (m.len == 0 || m.len > 64)
        continue;

      const uint8_t cmd = m.data[0];
      const uint8_t payLen = (m.len > 1) ? static_cast<uint8_t>(m.len - 1) : 0;

      uint8_t minL = 0, maxL = 0;
      if (!cmd_len_bounds(cmd, minL, maxL))
        continue; // unknown cmd
      if (payLen < minL || payLen > maxL)
        continue; // malformed len

      const uint8_t idx = findIndex(cmd);
      if (idx == 0xFF || s_table[idx].fn == nullptr)
        continue;

      // Make a bounded local copy so handlers can't overrun
      uint8_t payloadCopy[64];
      if (payLen)
      {
        // data starts at m.data[1]
        memcpy(payloadCopy, &m.data[1], payLen);
      }

      CmdFrame cf;
      cf.id = id;
      cf.cmd = cmd;
      cf.payload = payLen ? payloadCopy : nullptr;
      cf.len = payLen;

#if defined(__cpp_exceptions)
      try
      {
        s_table[idx].fn(cf);
      }
      catch (...)
      {
      }
#else
      s_table[idx].fn(cf);
#endif
    }
  }

  bool registerHandler(uint8_t cmd, Handler fn)
  {
    const uint8_t idx = findIndex(cmd);
    if (idx != 0xFF)
    {
      s_table[idx].fn = fn;
      return true;
    }

    if (s_count >= CANCMD_MAX_HANDLERS)
      return false;
    s_table[s_count].cmd = cmd;
    s_table[s_count].fn = fn;
    ++s_count;
    return true;
  }

  void setIdFilter(uint16_t filter, uint16_t mask)
  {
    s_idFilter = static_cast<uint16_t>(filter & 0x7FF);
    s_idMask = static_cast<uint16_t>(mask & 0x7FF);
  }

  void setDefaultTxId(uint16_t id)
  {
    s_defaultTxId = static_cast<uint16_t>(id & 0x7FF);
  }

  // -----------------------------------------------------------------------------
  // TX helpers
  // -----------------------------------------------------------------------------
  bool send(uint16_t id, uint8_t cmd, const void *data, uint8_t len)
  {
    uint8_t buf[64] = {0};
    buf[0] = cmd;

    if (len > 0 && data != nullptr)
    {
      const uint8_t maxPay = 63;
      if (len > maxPay)
        len = maxPay;
      memcpy(&buf[1], data, len);
    }

    const uint8_t rawLen = static_cast<uint8_t>(1 + len);
    CANFDMessage m;
    m.id = static_cast<uint32_t>(id & 0x7FF);
    m.len = normalizeFdLen(rawLen);
    memset(m.data, 0, sizeof(m.data));
    memcpy(m.data, buf, rawLen);

    const uint32_t st = fdcan1.tryToSendReturnStatusFD(m);
    return (st == 0);
  }

  bool sendDefault(uint8_t cmd, const void *data, uint8_t len)
  {
    return send(s_defaultTxId, cmd, data, len);
  }

  bool sendF32(uint16_t id, uint8_t cmd, float value)
  {
    return send(id, cmd, &value, sizeof(value));
  }
  bool sendF32(uint8_t cmd, float value)
  {
    return send(s_defaultTxId, cmd, &value, sizeof(value));
  }

  bool sendI32(uint16_t id, uint8_t cmd, int32_t value)
  {
    return send(id, cmd, &value, sizeof(value));
  }
  bool sendI32(uint8_t cmd, int32_t value)
  {
    return send(s_defaultTxId, cmd, &value, sizeof(value));
  }

  bool sendU16(uint16_t id, uint8_t cmd, uint16_t value)
  {
    return send(id, cmd, &value, sizeof(value));
  }
  bool sendU16(uint8_t cmd, uint16_t value)
  {
    return send(s_defaultTxId, cmd, &value, sizeof(value));
  }

  bool sendU8(uint16_t id, uint8_t cmd, uint8_t value)
  {
    return send(id, cmd, &value, sizeof(value));
  }
  bool sendU8(uint8_t cmd, uint8_t value)
  {
    return send(s_defaultTxId, cmd, &value, sizeof(value));
  }

  bool sendStruct(uint16_t id, uint8_t cmd, const void *p, size_t n)
  {
    if (p == nullptr)
      return false;
    if (n > 63)
      n = 63;
    return send(id, cmd, p, static_cast<uint8_t>(n));
  }

}