#include "Encoder.h"
#include <Wire.h>
#include <SPI.h>
#include <math.h>

Encoder::Encoder(uint16_t cpr)
: _as(), _as5048a(PB2, false), _cpr(cpr), _useAS5600(true) {}

bool Encoder::begin(uint8_t SDA, uint8_t SCL, bool useAS5600, uint8_t as5048_cs) {
  _useAS5600 = useAS5600;
  _as5048_cs = as5048_cs;

  // I2C for AS5600
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.setClock(400000);
  Wire.begin();

  // SPI for AS5048A
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);

  // Reconstruct AS5048A with the right CS if needed
  _as5048a = AS5048A(_as5048_cs, false);

  // Only init what we need
  if (_useAS5600) {
    if (!_as.begin()) return false;
  } else {
    _as5048a.begin();
  }

  _lastRaw = _useAS5600 ? _as.readAngle() : _as5048a.getRawRotation();
  _cont    = 0;
  _velCps  = 0.0;
  return true;
}

void Encoder::calibrateZero() {
  _lastRaw = _useAS5600 ? _as.readAngle() : _as5048a.getRawRotation();
  _cont    = 0;
  _velCps  = 0.0;
}

void Encoder::setInvert(bool inv) { _invert = inv; }

void Encoder::setVelAlpha(float a) {
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  _alpha = a;
}

void Encoder::update(double dt_s) {
  if (dt_s <= 0.0) return;

  const uint16_t raw = _useAS5600 ? _as.readAngle() : _as5048a.getRawRotation();

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

double Encoder::angle(Units u) const {
  const double s = _invert ? -1.0 : 1.0;
  switch (u) {
    case Degrees:   return s * _cont * (360.0 / _cpr);
    case Radians:   return s * _cont * (2.0 * M_PI / _cpr);
    case Rotations: return s * static_cast<double>(_cont) / _cpr;
  }
  return 0.0;
}

double Encoder::velocity(Units u) const {
  const double s = _invert ? -1.0 : 1.0;
  switch (u) {
    case Degrees:   return s * _velCps * (360.0 / _cpr);
    case Radians:   return s * _velCps * (2.0 * M_PI / _cpr);
    case Rotations: return s * _velCps / _cpr;
  }
  return 0.0;
}

bool Encoder::magnetPresent() {
  return _useAS5600 ? _as.detectMagnet() : true; // AS5048A has no equivalent
}
bool Encoder::magnetTooWeak()   { return _useAS5600 ? _as.magnetTooWeak()   : false; }
bool Encoder::magnetTooStrong() { return _useAS5600 ? _as.magnetTooStrong() : false; }
bool Encoder::magnetWrong()     { return _useAS5600 ? (!magnetPresent() || magnetTooWeak() || magnetTooStrong()) : false; }

uint16_t Encoder::rawCounts() { return _useAS5600 ? _as.readAngle() : _as5048a.getRawRotation(); }
uint16_t Encoder::cpr() const { return _cpr; }