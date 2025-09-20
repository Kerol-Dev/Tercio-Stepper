#pragma once
#include <stdint.h>

// ===== API surface =====
struct CanControlCallbacks {
  // Provide only the ones you use; leave others nullptr if unused.
  void (*setTargetAngleRad)(double rad)            = nullptr;
  void (*setMaxRps)(float rps)                     = nullptr;
  void (*setPid)(float kp, float ki, float kd)     = nullptr;
  void (*setCurrentLimit)(float amps)              = nullptr;
};

// Call these from your app:
void CanCmd_Begin();                               // init FDCAN1 (PA11 RX, PA12 TX), 500k/2.5M FD
void CanCmd_Poll();                                // poll & dispatch received frames
void CanCmd_SetCallbacks(const CanControlCallbacks& cb);

// ===== Expose opcodes (useful for your sender, docs, etc.) =====
enum class CanCmd : uint8_t {
  SET_TARGET_ANGLE = 0x01, // payload: float32 radians (LE)
  SET_MAX_RPS      = 0x02, // payload: float32 rps (LE)
  SET_PID          = 0x03, // payload: float32 kp, float32 ki, float32 kd (LE)
  SET_CURRENT      = 0x04, // payload: float32 amps (LE)
  // Add new commands here …
};