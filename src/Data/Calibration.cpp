#include "Calibration.h"
#include "EncoderAS5600.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"

static inline void jog(StepperControl &stepgen, EncoderAS5600 &enc, double rps, uint32_t ms)
{
  stepgen.setSpeedRPS(rps);
  const uint32_t t0 = millis();
  while (millis() - t0 < ms)
  {
    enc.update(1);
  }
  stepgen.stop();
}

bool Calibrate_EncoderDirection(EncoderAS5600 &enc,
                                StepperControl &stepgen,
                                AxisController &axis,
                                AxisConfig &cfg,
                                HardwareSerial &dbg,
                                double test_rps,
                                uint32_t jog_ms)
{
  dbg.println("[CAL] Encoder direction...");

  stepgen.enable();
  stepgen.setMicrostep(cfg.microsteps);
  axis.setMicrosteps(cfg.microsteps);

  const double start_deg = enc.angle() * RAD_TO_DEG; // encoder.angle() returns radians by default

  jog(stepgen, enc, +test_rps, jog_ms);

  const double end_deg = enc.angle() * RAD_TO_DEG;
  const double delta = end_deg - start_deg;

  dbg.print("[CAL] CW delta = ");
  dbg.print(delta, 2);
  dbg.println(" deg");

  const bool inverted = (delta < 0.0);
  cfg.encInvert = inverted;
  enc.setInvert(inverted);

  dbg.print("[CAL] invert = ");
  dbg.println(inverted ? "TRUE" : "FALSE");


  jog(stepgen, enc, -test_rps, jog_ms);
  cfg.encZeroCounts = 0; // if you prefer raw: cfg.encZeroCounts = enc.rawCounts();
  enc.calibrateZero();

  dbg.println("[CAL] Done.");
  return true;
}