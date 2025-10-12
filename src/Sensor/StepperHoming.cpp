#include "StepperHoming.h"
#include "Main.h"

bool StepperHoming::begin(const HomingConfig& cfg) {
  _cfg = cfg;
  pinMode(cfg.inMinPin, cfg.minActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(cfg.inMaxPin, cfg.maxActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  return true;
}

bool StepperHoming::readPin(uint8_t pin, bool activeLow) const {
  bool raw = digitalRead(pin);
  return activeLow ? !raw : raw;
}

void StepperHoming::update() {
  _minTrig = readPin(_cfg.inMinPin, _cfg.minActiveLow);
  _maxTrig = readPin(_cfg.inMaxPin, _cfg.maxActiveLow);
}

bool StepperHoming::home(SetVelFn setVel, StopFn stop, EncoderAS5600 enc, SetFn setZero, bool seekToMin) {
  const uint32_t t0 = millis();
  auto timeout = [&]() { return (millis() - t0) > _cfg.timeoutMs; };

  // Pick which switch to use
  const bool useMin = seekToMin;

  // Start moving toward the chosen limit
  float vel = useMin ? -fabsf(_cfg.seekSpeed) : fabsf(_cfg.seekSpeed);
  setVel(vel);

  while (!timeout()) {
    update();
    enc.update(0.01);
    if ((useMin && _minTrig) || (!useMin && _maxTrig)) break;
    delay(1);
  }
  stop();
  if (timeout()) return false;

  delay(2000);
  
  // Back off
  enc.update(0.01); // update once to avoid large jump
  double start = enc.angle();
  setVel(-vel);
  while (true) {
    enc.update(0.01);
    double d = fabsf(enc.angle() - start);
    if (d >= _cfg.backoffOffset) break;
    delay(1);
  }
  stop();

  DBG_PRINTLN("Homing: backoff done");
  // Zero encoder
  setZero(0.0f);
  return true;
}