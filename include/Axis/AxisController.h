#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include "EncoderAS5600.h"
#include "ConfigStore.h"
#include "StepperControl.h"

// Small PID used by AxisController
struct SimplePID
{
  double kp{3.0}, ki{0.0}, kd{0.0};
  double ePrev{0.0}, integ{0.0};
  double outMin{-10.0}, outMax{+10.0};

  void setTunings(double Kp, double Ki, double Kd);
  double compute(double err, double dt);
};

// Axis position controller (pos -> vel PID)
class AxisController
{
public:
  struct ExtPins
  {
    int step{-1}, dir{-1}, en{-1};
    bool enActiveLow{true};
  };
  struct TmcConfig
  {
    uint16_t mA{1200};
    uint8_t toff{5};
    uint8_t blank{24};
    bool stealth{true};
    double spreadSwitchRPS{8.0};
  };

  AxisController(EncoderAS5600 &enc,
                 StepperControl &stepgen,
                 TMC2209Stepper &tmc,
                 AxisConfig &cfg);

  void begin();

  void configureDriver(const TmcConfig &c);

  void setMicrosteps(uint16_t micro);
  void setFullSteps(uint16_t fullSteps);
  uint16_t microsteps() const;

  void setPID(double Kp, double Ki, double Kd);
  void setLimits(double maxRPS);
  void setTargetAngleRad(double r);
  double targetAngleRad() const;

  void setExternalMode(bool enabled);
  bool externalMode() const;
  void attachExternal(const ExtPins &p);

  void update(double dt);

  void setSpreadSwitchRPS(double rps);


private:
  static void onStepISR();

  EncoderAS5600 &_enc;
  TMC2209Stepper &_tmc;
  AxisConfig &_cfg;
  StepperControl &_stepgen;

  SimplePID _pid{};
  ExtPins _ext{};
  bool _externalMode{false};

  volatile int32_t _extStepPulses{0};
  static AxisController *s_owner;
  
  double _ustepAngleRad{(2.0 * PI) / (200.0 * 16.0)};
  double _targetRad{0.0};
  double _cmdRPS{0.0};
  double _spreadSwitchRPS{8.0};
};