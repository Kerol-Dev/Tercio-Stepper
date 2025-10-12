#pragma once
#include <Arduino.h>
#include <ConfigStore.h>

class HardwareTimer;

class StepperControl
{
public:
    StepperControl(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, AxisConfig &cfg);

    void begin();

    void enable();
    void disable();

    void setDir(bool cw);

    // Speed control
    void setStepRate(double sps);
    void stop();
    void setSpeedRPS(double rps);

    double stepsPerRev();

private:
    uint8_t _stepPin,
        _dirPin,
        _enPin;
    AxisConfig &_cfg;

    HardwareTimer *_tim{nullptr};
};
