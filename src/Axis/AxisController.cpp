#include "AxisController.h"
#include <cmath>

// ---------- SimplePID ----------
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
}
double SimplePID::compute(double err, double dt)
{
  integ += err * ki * dt;
  if (integ > outMax)
    integ = outMax;
  if (integ < outMin)
    integ = outMin;
  const double deriv = (dt > 0.0) ? (err - ePrev) / dt : 0.0;
  ePrev = err;
  double u = kp * err + integ + kd * deriv;
  if (u > outMax)
    u = outMax;
  if (u < outMin)
    u = outMin;
  return u;
}

// ---------- AxisController ----------
AxisController *AxisController::s_owner = nullptr;

AxisController::AxisController(EncoderAS5600 &enc,
                               StepperControl &stepgen,
                               TMC2209Stepper &tmc,
                               uint16_t fullSteps,
                               uint16_t micro)
    : _enc(enc), _stepgen(stepgen), _tmc(tmc), _fullSteps(fullSteps)
{
  setMicrosteps(micro);
}

void AxisController::begin()
{
  _enc.begin(PB7, PA15);
  _enc.calibrateZero();
  _stepgen.begin();
  _stepgen.enable();
}

void AxisController::configureDriver(const TmcConfig &c)
{
  _tmc.begin();
  _tmc.shaft(true);
  _tmc.toff(c.toff);
  _tmc.blank_time(c.blank);
  _tmc.rms_current(c.mA);
  _tmc.pwm_autoscale(true);
  _tmc.intpol(true);
  _tmc.TPOWERDOWN(0);
  _tmc.SGTHRS(0);

  if (c.spreadAlways)
  {
    _tmc.en_spreadCycle(true);
  }
  else
  {
    _tmc.en_spreadCycle(false);
    setSpreadSwitchRPS(c.spreadSwitchRPS);
  }
  if (c.stealth)
  {
    _tmc.pwm_autograd(true);
  }

  setMicrosteps(_micro);
}

void Axiscontroller::setFullSteps(uint16_t fullSteps)
{
  if (fullSteps == 0)
    fullSteps = 200;
  _fullSteps = fullSteps;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_fullSteps * _micro);
  setSpreadSwitchRPS(_spreadSwitchRPS);
}

void AxisController::setMicrosteps(uint16_t micro)
{
  if (micro == 0)
    micro = 1;
  _micro = micro;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_fullSteps * _micro);
  _stepgen.setMicrostep(_micro);
  _tmc.microsteps(_micro);
  setSpreadSwitchRPS(_spreadSwitchRPS);
}

uint16_t AxisController::microsteps() const { return _micro; }

void AxisController::setPID(double Kp, double Ki, double Kd)
{
  _pid.setTunings(Kp, Ki, Kd);
  _pid.setOutputLimits(-_lim.maxRPS, _lim.maxRPS);
}

void AxisController::setLimits(double maxRPS)
{
  _lim.maxRPS = maxRPS;
  _pid.setOutputLimits(-_lim.maxRPS, _lim.maxRPS);
}

void AxisController::setTargetAngleRad(double r) { _targetRad = r; }
double AxisController::targetAngleRad() const { return _targetRad; }

void AxisController::setExternalMode(bool enabled) { _externalMode = enabled; }
bool AxisController::externalMode() const { return _externalMode; }

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
  // External STEP/DIR/EN → target adjust + gating
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
      else
      {
        _stepgen.enable();
      }
    }
  }

  _enc.update(dt);

  const double pos = _enc.angle(); // radians
  const double err = _targetRad - pos;

  const double rpsDes = _pid.compute(err, dt);
  _cmdRPS = rpsDes;
  if (_cmdRPS > _lim.maxRPS)
    _cmdRPS = _lim.maxRPS;
  if (_cmdRPS < -_lim.maxRPS)
    _cmdRPS = -_lim.maxRPS;

  _stepgen.setSpeedRPS(_cmdRPS);
  _stepgen.service();
}

AxisController::Limits &AxisController::limits() { return _lim; }

void AxisController::setSpreadSwitchRPS(double rps)
{
  _spreadSwitchRPS = (rps <= 0.0) ? 0.1 : rps;
  const uint32_t fclk = 12000000UL; // typical TMC2209 internal clock
  const double usteps_per_rev = static_cast<double>(_fullSteps) * static_cast<double>(_micro);
  const double f_step = _spreadSwitchRPS * usteps_per_rev; // microsteps per second
  uint32_t tpwm = static_cast<uint32_t>(std::lround(static_cast<double>(fclk) / f_step));
  if (tpwm < 1)
    tpwm = 1;
  if (tpwm > 0xFFFFF)
    tpwm = 0xFFFFF;
  _tmc.TPWMTHRS(tpwm);
}

// ISR
void AxisController::onStepISR()
{
  if (s_owner)
    s_owner->_extStepPulses++;
}