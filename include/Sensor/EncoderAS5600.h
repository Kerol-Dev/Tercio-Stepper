#pragma once
#include <Arduino.h>
#include <AS5600.h>

class EncoderAS5600 {
public:
  enum Units : uint8_t { Degrees, Radians, Rotations };

  explicit EncoderAS5600(uint16_t cpr = 4096);

  bool  begin(uint8_t sda, uint8_t scl, uint32_t hz = 400000);
  void  calibrateZero();

  void  setInvert(bool inv);
  void  setVelAlpha(float a);

  void  update(double dt_s); // called regularly with measured dt (seconds)

  double angle(Units u = Radians) const;
  double velocity(Units u = Radians) const;

  bool  magnetPresent();
  bool  magnetTooWeak();
  bool  magnetTooStrong();
  bool  magnetWrong();

  uint16_t rawCounts();
  uint16_t cpr() const;

private:
  AS5600   _as;
  const uint16_t _cpr;

  uint16_t _lastRaw{0};
  int32_t  _cont{0};      // continuous counts since last zero
  double   _velCps{0.0};  // counts per second (LPF)

  float    _alpha{0.5f};  // velocity LPF coefficient
  bool     _invert{false};
};