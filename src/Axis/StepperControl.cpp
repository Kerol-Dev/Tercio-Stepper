#include "StepperControl.h"
#include <HardwareTimer.h>
#include <algorithm>

// ----------------------------------------------------------------------------
// StepperControl
// ----------------------------------------------------------------------------
namespace
{
  constexpr uint32_t kIdleHz = 1;       // PWM when "stopped"
  constexpr uint8_t kPwmChannel = 2;    // TIM1 CH2 -> PB14
  constexpr uint8_t kPwmDutyPct = 50;   // 50% step pulse duty
  constexpr double kMinActiveSps = 0.5; // below this, treat as stopped
}

StepperControl::StepperControl(uint8_t stepPin,
                               uint8_t dirPin,
                               uint8_t enPin,
                               AxisConfig cfg)
    : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin),
      _cfg(cfg) {}

void StepperControl::begin()
{
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  disable();

  _tim = new HardwareTimer(TIM1);
  _tim->setMode(kPwmChannel, TIMER_OUTPUT_COMPARE_PWM1, _stepPin);
  _tim->setOverflow(kIdleHz, HERTZ_FORMAT);
  _tim->setCaptureCompare(kPwmChannel, kPwmDutyPct, PERCENT_COMPARE_FORMAT);
  _tim->resume();
}

void StepperControl::enable() { digitalWrite(_enPin, LOW); }
void StepperControl::disable() { digitalWrite(_enPin, HIGH); }

void StepperControl::setDir(bool cw)
{
  digitalWrite(_dirPin, cw ? HIGH : LOW);
}

void StepperControl::setStepRate(double sps)
{
  if (!_tim)
    return;

  if (sps < kMinActiveSps)
  {
    _tim->pause();
    _tim->setOverflow(kIdleHz, HERTZ_FORMAT);
    _tim->setCaptureCompare(kPwmChannel, kPwmDutyPct, PERCENT_COMPARE_FORMAT);
    _tim->resume();
    return;
  }

  const uint32_t hz = static_cast<uint32_t>(std::max(1.0, sps));
  _tim->setOverflow(hz, HERTZ_FORMAT);
  _tim->setCaptureCompare(kPwmChannel, kPwmDutyPct, PERCENT_COMPARE_FORMAT);
  _tim->resume();
}

void StepperControl::stop()
{
  setStepRate(0.0);
}

void StepperControl::setSpeedRPS(double rps)
{
  const double sps = std::abs(rps) * stepsPerRev();
  setStepRate(sps);
  setDir(rps >= 0.0);
}

double StepperControl::stepsPerRev() const
{
  return static_cast<double>(_cfg.stepsPerRev) * static_cast<double>(_cfg.microsteps);
}