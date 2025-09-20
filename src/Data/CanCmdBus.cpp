// -----------------------------------------------------------------------------
// STM32G431 — CAN-FD Command Receiver on FDCAN1 (PA11 RX, PA12 TX)
// Control frames on ID 0x123 (FD, 500 kbps nominal, 2.5 Mbps data (x5))
// Does not initialize Debug (assumes you have a global 'Print& Debug' in Main.h)
// IMPORTANT: Include <ACANFD_STM32.h> ONCE from your main .ino,
//            and include <ACANFD_STM32_from_cpp.h> here.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ACANFD_STM32.h>
#include <ACANFD_STM32_Settings.h>
#include "CanCmdBus.h"
#include "Main.h"   // provides: extern Print& Debug;

// ===== Configuration =====
static constexpr uint32_t CAN_CTRL_ID       = 0x123;
static constexpr uint8_t  CAN_CMD_PROTO_VER = 1;

// Pins (AF9)
static constexpr uint8_t FDCAN1_RX_PIN = PA11;
static constexpr uint8_t FDCAN1_TX_PIN = PA12;

// ===== Internal state =====
static CanControlCallbacks s_cb;

// ===== Helpers (safe LE reads) =====
static inline bool rdU8 (const uint8_t* b, size_t len, size_t off, uint8_t& out) {
  if (off + 1 > len) return false; out = b[off]; return true;
}
static inline bool rdF32(const uint8_t* b, size_t len, size_t off, float& out) {
  if (off + 4 > len) return false; memcpy(&out, b + off, 4); return true;
}

// ===== Dispatcher =====
static void handleCommand(const CANFDMessage& m) {
  if (m.id != CAN_CTRL_ID || m.len < 2) return; // Need opcode + version

  const uint8_t* d = m.data;
  const size_t   n = m.len;

  uint8_t opcode = 0, ver = 0;
  if (!rdU8(d, n, 0, opcode) || !rdU8(d, n, 1, ver)) return;
  if (ver != CAN_CMD_PROTO_VER) {
    Debug.println("[CAN] Ignoring cmd: version mismatch");
    return;
  }

  Debug.print("[CAN] Cmd opcode=0x"); Debug.println(opcode, HEX);

  switch (static_cast<CanCmd>(opcode)) {
    case CanCmd::SET_TARGET_ANGLE: {
      float rad;
      if (rdF32(d, n, 2, rad) && s_cb.setTargetAngleRad) {
        Debug.print("  SET_TARGET_ANGLE "); Debug.println(rad, 6);
        s_cb.setTargetAngleRad(static_cast<double>(rad));
      }
    } break;

    case CanCmd::SET_MAX_RPS: {
      float rps;
      if (rdF32(d, n, 2, rps) && s_cb.setMaxRps) {
        Debug.print("  SET_MAX_RPS "); Debug.println(rps, 6);
        s_cb.setMaxRps(rps);
      }
    } break;

    case CanCmd::SET_PID: {
      float kp, ki, kd;
      if (rdF32(d, n, 2, kp) && rdF32(d, n, 6, ki) && rdF32(d, n, 10, kd) && s_cb.setPid) {
        Debug.print("  SET_PID kp="); Debug.print(kp, 6);
        Debug.print(" ki="); Debug.print(ki, 6);
        Debug.print(" kd="); Debug.println(kd, 6);
        s_cb.setPid(kp, ki, kd);
      }
    } break;

    case CanCmd::SET_CURRENT: {
      float amps;
      if (rdF32(d, n, 2, amps) && s_cb.setCurrentLimit) {
        Debug.print("  SET_CURRENT "); Debug.println(amps, 6);
        s_cb.setCurrentLimit(amps); // units: amperes (e.g., 1.4 = 1.4 A)
      }
    } break;

    default:
      // Unknown opcode: ignore (extensible)
      Debug.println("  Unknown opcode; ignoring");
      break;
  }
}

// ===== Public functions =====
void CanCmd_SetCallbacks(const CanControlCallbacks& cb) { s_cb = cb; }

void CanCmd_Begin() {
  // 500 kbps nominal, 2.5 Mbps data (factor x5)
  ACANFD_STM32_Settings settings(500000, DataBitRateFactor::x5);
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD; // real bus
  settings.mRxPin = FDCAN1_RX_PIN;
  settings.mTxPin = FDCAN1_TX_PIN;

  // Start FDCAN (ignore error code here; app may decide to assert)
  (void)fdcan1.beginFD(settings);
}

void CanCmd_Poll() {
  CANFDMessage m;
  while (fdcan1.receiveFD0(m)) {
    handleCommand(m);
  }
}