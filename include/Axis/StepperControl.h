#pragma once
#include <Arduino.h>

class HardwareTimer;

/**
 * Lightweight STEP/DIR pulse generator.
 * STEP: TIM1 CH2 (PB14), DIR: PB15, EN: PB5 (active-low).
 */
class StepperControl {
public:
  StepperControl(uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                 uint16_t fullStepsPerRev = 200);

  void begin();

  void enable();              // EN low (active)
  void disable();             // EN high (inactive)

  void setMicrostep(uint16_t m);
  void setFullSteps(uint16_t fs);
  void setDir(bool cw);

  // Speed control
  void setStepRate(double sps);   // microsteps per second
  void stop();
  void setSpeedRPS(double rps);   // revolutions per second (signed)

  void service();                 // reserved (no-op)

  uint16_t microstep() const;
  double   stepsPerRev() const;

private:
  uint8_t  _stepPin, _dirPin, _enPin;
  uint16_t _fullSteps;
  uint16_t _micro{16};

  HardwareTimer* _tim{nullptr};
};
