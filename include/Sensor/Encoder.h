#pragma once
#include <Arduino.h>
#include <AS5600.h>
#include <AS5048A.h>

class Encoder {
public:
  enum Units : uint8_t { Degrees, Radians, Rotations };

  explicit Encoder(uint16_t cpr = 4096);

  bool  begin(uint8_t SDA, uint8_t SCL, bool useAS5600, uint8_t as5048_cs);
  void  calibrateZero();

  void  setInvert(bool inv);
  void  setVelAlpha(float a);

  void  update(double dt_s);

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
  AS5048A  _as5048a;       // will be constructed with a real CS
  uint16_t _cpr;
  bool     _useAS5600;     // <-- not const anymore
  uint8_t  _as5048_cs{PB2};

  uint16_t _lastRaw{0};
  int32_t  _cont{0};
  double   _velCps{0.0};

  float    _alpha{0.5f};
  bool     _invert{false};
};