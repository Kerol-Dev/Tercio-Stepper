#pragma once
#include <Arduino.h>

class Encoder;
class StepperControl;
class AxisController;
struct AxisConfig;

bool Calibrate_EncoderDirection(Encoder& enc,
                                StepperControl& stepgen,
                                AxisController& axis,
                                AxisConfig& cfg,
                                double test_rps = 2,
                                uint32_t jog_ms = 1000);