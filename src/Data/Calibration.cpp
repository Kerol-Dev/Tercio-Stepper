#include "Calibration.h"
#include "EncoderAS5600.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"

// -----------------------------------------------------------------------------
// Lightweight logging helpers (use the provided 'dbg' sink, but easy to mute)
// -----------------------------------------------------------------------------
#define CAL_LOG(dbg, msg)        do { (dbg).println(msg); } while (0)
#define CAL_LOGF(dbg, fmt, ...)  do { (dbg).printf((fmt), __VA_ARGS__); } while (0)

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------
static inline void jog(StepperControl& stepgen, EncoderAS5600& enc, double rps, uint32_t ms)
{
  stepgen.setSpeedRPS(rps);
  const uint32_t t0_ms = millis();
  uint32_t last_us = micros();

  while ((millis() - t0_ms) < ms) {
    const uint32_t now_us = micros();
    const double dt_sec = (now_us - last_us) * 1e-6;
    last_us = now_us;
    enc.update(dt_sec);
  }

  stepgen.stop();
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool Calibrate_EncoderDirection(EncoderAS5600& enc,
                                StepperControl& stepgen,
                                AxisController& axis,
                                AxisConfig& cfg,
                                HardwareSerial& dbg,
                                double test_rps,
                                uint32_t jog_ms)
{
  // CAL_LOG(dbg, "[CAL] Encoder direction...");

  // Ensure known state
  enc.setInvert(false);
  stepgen.enable();

  // Keep microstep settings aligned
  stepgen.setMicrostep(cfg.microsteps);
  axis.setMicrosteps(cfg.microsteps);

  // Measure CW delta (encoder.angle() returns radians)
  const double start_deg = enc.angle() * RAD_TO_DEG;
  jog(stepgen, enc, +test_rps, jog_ms);
  const double end_deg = enc.angle() * RAD_TO_DEG;
  const double delta_deg = end_deg - start_deg;

  // CAL_LOGF(dbg, "[CAL] CW delta = %.2f deg\r\n", delta_deg);

  // Determine inversion: if CW jog reduced angle, encoder is inverted
  const bool inverted = (delta_deg < 0.0);
  cfg.encInvert = inverted;
  enc.setInvert(inverted);

  // CAL_LOGF(dbg, "[CAL] invert = %s\r\n", inverted ? "TRUE" : "FALSE");

  // Return to start vicinity and zero
  jog(stepgen, enc, -test_rps, jog_ms);
  enc.calibrateZero();

  // CAL_LOG(dbg, "[CAL] Done.");
  return true;
}