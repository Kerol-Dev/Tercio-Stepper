#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include "EncoderAS5600.h"
#include "StepperControl.h"

// Small PID used by AxisController
struct SimplePID {
  double kp{3.0}, ki{0.0}, kd{0.0};
  double ePrev{0.0}, integ{0.0};
  double outMin{-10.0}, outMax{+10.0};

  void setTunings(double Kp, double Ki, double Kd);
  void setOutputLimits(double mn, double mx);
  double compute(double err, double dt);
};

// Axis position controller (pos -> vel PID) with optional external STEP/DIR/EN input
class AxisController {
public:
  struct Limits {
    double maxRPS{15.0};
  };
  struct ExtPins {
    int step{-1}, dir{-1}, en{-1};
    bool enActiveLow{true};
  };
  struct TmcConfig {
    uint16_t mA{1200};
    uint8_t  toff{4};
    uint8_t  blank{24};
    bool     spreadAlways{false};
    bool     stealth{true};
    double   spreadSwitchRPS{4.0};
  };

  AxisController(EncoderAS5600& enc,
                 StepperControl& stepgen,
                 TMC2209Stepper& tmc,
                 uint16_t fullSteps = 200,
                 uint16_t micro = 16);

  void begin();

  void configureDriver(const TmcConfig& c);

  void setMicrosteps(uint16_t micro);
  uint16_t microsteps() const;

  void setPID(double Kp, double Ki, double Kd);
  void setLimits(double maxRPS);
  void setTargetAngleRad(double r);
  double targetAngleRad() const;

  void setExternalMode(bool enabled);
  bool externalMode() const;
  void attachExternal(const ExtPins& p);

  // Call at high rate with fixed dt [s]
  void update(double dt);

  Limits& limits();

  void setSpreadSwitchRPS(double rps);

private:
  static void onStepISR();

  EncoderAS5600&   _enc;
  StepperControl&  _stepgen;
  TMC2209Stepper&  _tmc;

  const uint16_t _fullSteps;
  uint16_t _micro{16};
  double   _ustepAngleRad{(2.0 * PI) / (200.0 * 16.0)};

  Limits     _lim{};
  SimplePID  _pid{};
  ExtPins    _ext{};
  bool       _externalMode{false};

  volatile int32_t _extStepPulses{0};
  static AxisController* s_owner;

  double _targetRad{0.0};
  double _cmdRPS{0.0};
  double _spreadSwitchRPS{4.0};
};