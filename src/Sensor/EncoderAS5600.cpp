#include "EncoderAS5600.h"
#include <Wire.h>
#include <math.h>

EncoderAS5600::EncoderAS5600(uint16_t cpr) : _cpr(cpr) {}

bool EncoderAS5600::begin(uint8_t sda, uint8_t scl, uint32_t hz) {
  Wire.setSDA(sda);
  Wire.setSCL(scl);
  Wire.setClock(hz);
  Wire.begin();

  if (!_as.begin()) return false;

  _lastRaw = _as.readAngle();
  _cont    = 0;
  _velCps  = 0.0;
  return true;
}

void EncoderAS5600::calibrateZero() {
  _lastRaw = _as.readAngle();
  _cont    = 0;
  _velCps  = 0.0;
}

void EncoderAS5600::setInvert(bool inv) { _invert = inv; }

void EncoderAS5600::setVelAlpha(float a) {
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  _alpha = a;
}

void EncoderAS5600::update(double dt_s) {
  if (dt_s <= 0.0) return;

  const uint16_t raw = _as.readAngle();

  int16_t diff = static_cast<int16_t>(raw - _lastRaw);
  const int16_t half = static_cast<int16_t>(_cpr / 2);
  if (diff >  half) diff -= _cpr;
  if (diff < -half) diff += _cpr;

  _lastRaw = raw;

  const int32_t prevCont = _cont;
  _cont += diff;

  const double inst_cps = static_cast<double>(_cont - prevCont) / dt_s;
  _velCps = (1.0 - _alpha) * _velCps + _alpha * inst_cps;
}

double EncoderAS5600::angle(Units u) const {
  const double s = _invert ? -1.0 : 1.0;
  switch (u) {
    case Degrees:   return s * _cont * (360.0 / _cpr);
    case Radians:   return s * _cont * (2.0 * M_PI / _cpr);
    case Rotations: return s * static_cast<double>(_cont) / _cpr;
  }
  return 0.0;
}

double EncoderAS5600::velocity(Units u) const {
  const double s = _invert ? -1.0 : 1.0;
  switch (u) {
    case Degrees:   return s * _velCps * (360.0 / _cpr);
    case Radians:   return s * _velCps * (2.0 * M_PI / _cpr);
    case Rotations: return s * _velCps / _cpr;
  }
  return 0.0;
}

bool EncoderAS5600::magnetPresent()   { return _as.detectMagnet(); }
bool EncoderAS5600::magnetTooWeak()   { return _as.magnetTooWeak(); }
bool EncoderAS5600::magnetTooStrong() { return _as.magnetTooStrong(); }
bool EncoderAS5600::magnetWrong()     { return !magnetPresent() || magnetTooWeak() || magnetTooStrong(); }

uint16_t EncoderAS5600::rawCounts() { return _as.readAngle(); }
uint16_t EncoderAS5600::cpr() const { return _cpr; }