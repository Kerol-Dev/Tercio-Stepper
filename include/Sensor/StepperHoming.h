#pragma once
#include <Arduino.h>
#include <functional>
#include <EncoderAS5600.h>
#include "AxisController.h"

struct HomingConfig {
  uint8_t inMinPin;       // IN1 pin
  uint8_t inMaxPin;       // IN2 pin
  bool minActiveLow;      // true = active low, false = active high
  bool maxActiveLow;
  float seekSpeed;        // units/sec toward limit
  uint32_t timeoutMs;     // safety timeout
  float backoffOffset;    // distance to move off the switch
};

class StepperHoming {
public:
  using SetVelFn = std::function<void(float)>;  // set velocity
  using StopFn   = std::function<void(void)>;   // stop motor
  using ReadFn   = std::function<double(void)>;  // read position
  using SetFn    = std::function<void(float)>;  // set position

  StepperHoming() = default;
  bool begin(const HomingConfig& cfg);

  // Perform homing. Returns true if successful.
  bool home(SetVelFn setVel, StopFn stop, EncoderAS5600 enc, AxisController con, AxisConfig axisCfg, SetFn setZero, bool seekToMin);

  // Check raw state
  bool minTriggered() const { return _minTrig; }
  bool maxTriggered() const { return _maxTrig; }

  void update();
private:
  HomingConfig _cfg{};
  bool _minTrig = false;
  bool _maxTrig = false;

  bool readPin(uint8_t pin, bool activeLow) const;
};