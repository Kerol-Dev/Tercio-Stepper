#include "Calibration.h"
#include "EncoderAS5600.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"

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
  // Ensure known state
  enc.setInvert(false);
  stepgen.enable();

  // Keep microstep settings aligned
  axis.setMicrosteps(cfg.microsteps);

  // Measure CW delta (encoder.angle() returns radians)
  const double start_deg = enc.angle() * RAD_TO_DEG;
  jog(stepgen, enc, +test_rps, jog_ms);
  const double end_deg = enc.angle() * RAD_TO_DEG;
  const double delta_deg = end_deg - start_deg;

  // Determine inversion: if CW jog reduced angle, encoder is inverted
  const bool inverted = (delta_deg < 0.0);
  cfg.encInvert = inverted;
  enc.setInvert(inverted);

  // Return to start vicinity and zero
  jog(stepgen, enc, -test_rps, jog_ms);
  enc.calibrateZero();
  cfg.calibratedOnce = true;

  return true;
}