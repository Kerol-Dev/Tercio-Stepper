#include "StepperControl.h"
#include <HardwareTimer.h>

StepperControl::StepperControl(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                               uint16_t fullStepsPerRev)
: _stepPin(stepPin)
, _dirPin(dirPin)
, _enPin(enPin)
, _fullSteps(fullStepsPerRev) {}

void StepperControl::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  disable();

  _tim = new HardwareTimer(TIM1);
  // TIM1_CH2 on PB14
  _tim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, _stepPin);
  _tim->setOverflow(1, HERTZ_FORMAT);                // 1 Hz idle
  _tim->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT); // 50% duty
  _tim->resume();
}

void StepperControl::enable()  { digitalWrite(_enPin, LOW);  }
void StepperControl::disable() { digitalWrite(_enPin, HIGH); }

void StepperControl::setMicrostep(uint16_t m) {
  if (m == 0) m = 1;
  _micro = m;
}

void StepperControl::setFullSteps(uint16_t fs) {
  if (fs == 0) fs = 200;
  _fullSteps = fs;
}

void StepperControl::setDir(bool cw) {
  digitalWrite(_dirPin, cw ? HIGH : LOW);
}

void StepperControl::setStepRate(double sps) {
  if (!_tim) return;

  if (sps < 0.5) {
    _tim->pause();
    _tim->setOverflow(1, HERTZ_FORMAT);
    _tim->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);
    _tim->resume();
    return;
  }
  _tim->setOverflow(static_cast<uint32_t>(sps), HERTZ_FORMAT);
  _tim->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);
  _tim->resume();
}

void StepperControl::stop() { setStepRate(0.0); }

void StepperControl::setSpeedRPS(double rps) {
  const double sps = fabs(rps) * stepsPerRev();
  setStepRate(sps);
  setDir(rps >= 0.0);
}

void StepperControl::service() {
  // reserved for future use
}

uint16_t StepperControl::microstep() const { return _micro; }
double   StepperControl::stepsPerRev() const { return static_cast<double>(_fullSteps) * _micro; }