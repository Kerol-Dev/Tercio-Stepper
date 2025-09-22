#pragma once
#include <Arduino.h>

class EncoderAS5600;
class StepperControl;
class AxisController;
struct AxisConfig;

bool Calibrate_EncoderDirection(EncoderAS5600& enc,
                                StepperControl& stepgen,
                                AxisController& axis,
                                AxisConfig& cfg,
                                HardwareSerial& dbg,
                                double test_rps = 0.5,
                                uint32_t jog_ms = 2000);