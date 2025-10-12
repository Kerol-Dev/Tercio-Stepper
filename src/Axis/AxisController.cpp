#include "AxisController.h"
#include "Main.h"

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
  _tmc.toff(5);
  _tmc.blank_time(c.blank);
  _tmc.rms_current(c.mA);
  _tmc.pwm_autoscale(true);
  _tmc.pwm_freq(2);
  _tmc.pwm_lim(10);
  _tmc.TCOOLTHRS(0);
  _tmc.hstrt(4);
  _tmc.hend(1);
  _tmc.tbl(2);
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
      DBG_PRINTF("[EXT] pulses=%ld dir=%d newTarget=%.4f rad\n",
                 (long)pulses, sgn, _targetRad);
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

  // ---- Guard against timing glitches: clamp dt used for acceleration ----
  // Use a sane control-window for accel math (e.g., 0.001–0.02 s)
  //  -> At rps2=2, this yields dv in [0.002 .. 0.04] rps per tick.
  const double dt_eff = std::clamp(dt, 0.001, 0.020);
  const double dv_max = _cfg.maxRPS2 * dt_eff;   // rps per cycle

  // ---------- ONLY ACCELERATE; DECELERATE INSTANTLY ----------
  double new_cmd;

  if (fabs(vel_target) > fabs(_cmdRPS) && (fabs(vel_target - _cmdRPS) > dv_max)) {
    // Need more speed magnitude -> accelerate toward vel_target, limited by dv_max
    const double sign = (vel_target > _cmdRPS) ? +1.0 : -1.0;
    new_cmd = _cmdRPS + sign * dv_max;
  } else {
    // Want less magnitude OR within dv_max -> instant decel / direct set
    new_cmd = vel_target;
  }

  const bool accel_phase = (fabs(new_cmd) > fabs(_cmdRPS));
  _cmdRPS = std::clamp(new_cmd,
                       -static_cast<double>(_cfg.maxRPS),
                       static_cast<double>(_cfg.maxRPS));
  const bool speed_cap = (new_cmd != _cmdRPS);

  _stepgen.setSpeedRPS(_cmdRPS);

  // ---------- Low-noise debug ----------
  static double last_cmd = NAN;
  static uint32_t last_ms = 0;
  const uint32_t now = millis();

  const float CMD_EPS = 0.05f;   // rps
  const uint32_t MIN_PERIOD_MS = 15;

  if ( (std::isnan(last_cmd) || fabs(_cmdRPS - last_cmd) > CMD_EPS || accel_phase || speed_cap)
       && (now - last_ms) >= MIN_PERIOD_MS )
  {
    DBG_PRINTLN(_cmdRPS);
    last_cmd = _cmdRPS;
    last_ms  = now;
  }
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