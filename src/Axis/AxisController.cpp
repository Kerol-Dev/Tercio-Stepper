#include "AxisController.h"

#include <algorithm>
#include <cmath>

// ============================================================================
// SimplePID
// ============================================================================
void SimplePID::setTunings(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void SimplePID::setOutputLimits(double mn, double mx)
{
  outMin = mn;
  outMax = mx;
  integ = std::clamp(integ, outMin, outMax);
}

double SimplePID::compute(double err, double dt)
{
  // I
  integ += err * ki * dt;
  integ = std::clamp(integ, outMin, outMax);

  // D
  const double deriv = (dt > 0.0) ? (err - ePrev) / dt : 0.0;
  ePrev = err;

  // P + I + D
  double u = kp * err + integ + kd * deriv;
  return std::clamp(u, outMin, outMax);
}

// ============================================================================
// AxisController
// ============================================================================
AxisController *AxisController::s_owner = nullptr;

AxisController::AxisController(EncoderAS5600 &enc,
                               StepperControl &stepgen,
                               TMC2209Stepper &tmc,
                               AxisConfig &cfg)
    : _enc(enc), _stepgen(stepgen), _tmc(tmc),
      _cfg(cfg)
{
}

void AxisController::begin()
{
  _enc.calibrateZero();

  _stepgen.begin();
  _stepgen.enable();
}

void AxisController::configureDriver(const TmcConfig &c)
{
  _tmc.begin();
  _tmc.shaft(false);
  _tmc.toff(c.toff);
  _tmc.blank_time(c.blank);
  _tmc.rms_current(c.mA);
  _tmc.pwm_autoscale(true);
  _tmc.intpol(true);

  // Keep coils energized; disable runtime features we don't use
  _tmc.TPOWERDOWN(0);
  _tmc.TCOOLTHRS(0);
  _tmc.freewheel(0);
  _tmc.SGTHRS(0);

  if (c.stealth)
  {
    _tmc.en_spreadCycle(false);
    _tmc.pwm_autograd(true);
  }
  else
  {
    _tmc.en_spreadCycle(true);
  }

  setMicrosteps(_cfg.microsteps);
}

void AxisController::setFullSteps(uint16_t fullSteps)
{
  _cfg.stepsPerRev = fullSteps ? fullSteps : 200;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_cfg.stepsPerRev * _cfg.microsteps);
  setSpreadSwitchRPS(_spreadSwitchRPS);
}

void AxisController::setMicrosteps(uint16_t micro)
{
  _cfg.microsteps = micro ? micro : 1;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_cfg.stepsPerRev * _cfg.microsteps);

  _tmc.microsteps(_cfg.microsteps);

  setSpreadSwitchRPS(_spreadSwitchRPS);
}

uint16_t AxisController::microsteps() const
{
  return _cfg.microsteps;
}

void AxisController::setPID(double Kp, double Ki, double Kd)
{
  _pid.setTunings(Kp, Ki, Kd);
}

void AxisController::setTargetAngleRad(double r)
{
  _targetRad = r;
}

double AxisController::targetAngleRad() const
{
  return _targetRad;
}

void AxisController::setExternalMode(bool enabled)
{
  _externalMode = enabled;
}

bool AxisController::externalMode() const
{
  return _externalMode;
}

void AxisController::attachExternal(const ExtPins &p)
{
  _ext = p;

  if (_ext.en >= 0)
  {
    pinMode(_ext.en, INPUT_PULLUP);
  }
  if (_ext.step >= 0)
  {
    pinMode(_ext.step, INPUT_PULLDOWN);
    s_owner = this;
    attachInterrupt(digitalPinToInterrupt(_ext.step), AxisController::onStepISR, RISING);
  }
}

void AxisController::update(double dt)
{
  if (_externalMode && _ext.step >= 0)
  {
    int32_t pulses = 0;
    noInterrupts();
    pulses = _extStepPulses;
    _extStepPulses = 0;
    interrupts();

    if (pulses != 0)
    {
      const int sgn = (_ext.dir >= 0) ? (digitalRead(_ext.dir) ? +1 : -1) : +1;
      _targetRad += static_cast<double>(pulses * sgn) * _ustepAngleRad;
    }
    if (_ext.en >= 0)
    {
      const bool enActive = _ext.enActiveLow ? (digitalRead(_ext.en) == LOW)
                                             : (digitalRead(_ext.en) == HIGH);
      if (!enActive)
      {
        _stepgen.stop();
        _stepgen.disable();
        _cmdRPS = 0.0;
        return;
      }
      _stepgen.enable();
    }
  }

  _enc.update(dt);

  const double pos = _enc.angle(); // radians
  const double err = _targetRad - pos;

  double vel_target = _pid.compute(err, dt);

  if (vel_target > _cmdRPS && fabs(vel_target - _cmdRPS) > (_cfg.maxRPS2 * dt))
  {
    _cmdRPS += (_cfg.maxRPS2 * dt);
  }
  else
    _cmdRPS = vel_target;

  _cmdRPS = std::clamp(_cmdRPS, -static_cast<double>(_cfg.maxRPS), static_cast<double>(_cfg.maxRPS));

  _stepgen.setSpeedRPS(_cmdRPS);
  _stepgen.service();
}

void AxisController::setSpreadSwitchRPS(double rps)
{
  _spreadSwitchRPS = (rps > 0.0) ? rps : 0.1;

  // TMC2209 TPWMTHRS calculation
  static constexpr uint32_t fclk = 12000000UL; // internal clock
  const double usteps_per_rev = static_cast<double>(_cfg.stepsPerRev) * static_cast<double>(_cfg.microsteps);
  const double f_step = _spreadSwitchRPS * usteps_per_rev; // microsteps per second
  uint32_t tpwm = (f_step > 0.0)
                      ? static_cast<uint32_t>(std::lround(static_cast<double>(fclk) / f_step))
                      : 0xFFFFF;

  tpwm = std::clamp<uint32_t>(tpwm, 1, 0xFFFFF);
  _tmc.TPWMTHRS(tpwm);
}

// ISR
void AxisController::onStepISR()
{
  if (s_owner)
  {
    s_owner->_extStepPulses++;
  }
}