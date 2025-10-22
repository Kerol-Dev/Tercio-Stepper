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

double SimplePID::compute(double err, double dt)
{
  // I
  integ += err * ki * dt;

  // D
  const double deriv = (dt > 0.0) ? (err - ePrev) / dt : 0.0;
  ePrev = err;

  // P + I + D
  double u = kp * err + integ + kd * deriv;
  return u;
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
  _tmc.vsense(false);

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
  if (_ext.dir >= 0)
  {
    pinMode(_ext.dir, INPUT_PULLDOWN);
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
  // ---------- External step/dir ----------
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

  // PID proposes velocity (rps)
  const double vel_target = _pid.compute(err, dt);

  const double dt_eff = std::clamp(dt, 0.001, 0.020);
  const double dv_max = _cfg.maxRPS2 * dt_eff; // rps per cycle

  double new_cmd;

  if (fabs(vel_target) > fabs(_cmdRPS) && (fabs(vel_target - _cmdRPS) > dv_max))
  {
    const double sign = (vel_target > _cmdRPS) ? +1.0 : -1.0;
    new_cmd = _cmdRPS + sign * dv_max;
  }
  else
  {
    new_cmd = vel_target;
  }

  const bool accel_phase = (fabs(new_cmd) > fabs(_cmdRPS));
  _cmdRPS = std::clamp(new_cmd,
                       -static_cast<double>(_cfg.maxRPS),
                       static_cast<double>(_cfg.maxRPS));
  const bool speed_cap = (new_cmd != _cmdRPS);

  if (fabs(_cmdRPS) <= 0.001)
    _stepgen.setSpeedRPS(0);
  else
    _stepgen.setSpeedRPS(_cmdRPS);
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