#include "StepperControl.h"
#include <HardwareTimer.h>
#include <Main.h>
#include <algorithm>

// ----------------------------------------------------------------------------
// StepperControl
// ----------------------------------------------------------------------------
namespace {
  constexpr uint8_t  kPwmChannel    = 2;      // TIM1 CH2 -> PB14
  constexpr double   kMinActiveSps  = 0.05;   // below this, treat as stopped (no pulses)
  constexpr uint32_t kMinARR        = 2;      // need >=2 ticks to get a 50% pulse cleanly
  constexpr uint32_t kMaxARR        = 65535;  // 16-bit advanced timer (TIM1) max ARR
  constexpr uint32_t kMinPSC        = 1;      // HardwareTimer takes the *factor* (PSC+1); 1 -> no prescale
  constexpr uint32_t kMaxPSC        = 65536;  // max prescale factor for safety
  constexpr uint8_t  kDutyPercent   = 50;     // 50% duty for clean STEP pulses
}

StepperControl::StepperControl(uint8_t stepPin,
                               uint8_t dirPin,
                               uint8_t enPin,
                               AxisConfig &cfg)
: _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin), _cfg(cfg) {}

void StepperControl::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  disable();

  _tim = new HardwareTimer(TIM1);
  _tim->setMode(kPwmChannel, TIMER_OUTPUT_COMPARE_PWM1, _stepPin);

  // Start paused, output idle (no pulses)
  _tim->pause();
  _tim->setPrescaleFactor(kMinPSC);
  _tim->setOverflow(kMaxARR, TICK_FORMAT);
  _tim->setCaptureCompare(kPwmChannel, 0, PERCENT_COMPARE_FORMAT); // 0% -> no pulses
  _tim->resume();
}

void StepperControl::enable()  { digitalWrite(_enPin, LOW);  }
void StepperControl::disable() { digitalWrite(_enPin, HIGH); }

void StepperControl::setDir(bool cw) {
  digitalWrite(_dirPin, cw ? HIGH : LOW);
}

void StepperControl::setStepRate(double sps) {
  if (!_tim) return;

  // Stop (no pulses) for tiny speeds
  if (std::abs(sps) < kMinActiveSps) {
    _tim->pause();
    // Keep output steady: 0% duty
    _tim->setCaptureCompare(kPwmChannel, 0, PERCENT_COMPARE_FORMAT);
    _tim->resume();
    return;
  }

  // Target STEP frequency (Hz)
  const double f_target = std::max(0.0, sps);

  const uint32_t f_tim = _tim->getTimerClkFreq();
  const double ticks_total = static_cast<double>(f_tim) / f_target;

  uint32_t presc = (ticks_total > static_cast<double>(kMaxARR))
                     ? static_cast<uint32_t>( (ticks_total + kMaxARR - 1.0) / kMaxARR )
                     : 1u;
  if (presc < kMinPSC) presc = kMinPSC;
  if (presc > kMaxPSC) presc = kMaxPSC;

  double arr_f = ticks_total / static_cast<double>(presc);
  uint32_t arr = static_cast<uint32_t>(std::clamp(arr_f, static_cast<double>(kMinARR),
                                                  static_cast<double>(kMaxARR)));
  if (arr < kMinARR)  arr = kMinARR;
  if (arr > kMaxARR)  arr = kMaxARR;

  // Apply (pause to avoid mid-update glitches)
  _tim->pause();
  _tim->setPrescaleFactor(presc);                 // prescaler = presc
  _tim->setOverflow(arr, TICK_FORMAT);    // period = ARR+1 ticks effectively
  _tim->setCaptureCompare(kPwmChannel, kDutyPercent, PERCENT_COMPARE_FORMAT); // 50% duty
  _tim->resume();
}

void StepperControl::stop() {
  setStepRate(0.0);
}

void StepperControl::setSpeedRPS(double rps) {
  const double sps = std::abs(rps) * stepsPerRev();
  setStepRate(sps);
  setDir(rps >= 0.0);
}

double StepperControl::stepsPerRev() {
  return static_cast<double>(_cfg.stepsPerRev) * static_cast<double>(_cfg.microsteps);
}