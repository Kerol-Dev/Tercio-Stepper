#include <Arduino.h>
#include <AS5600.h>
#include <TMCStepper.h>

#include "EncoderAS5600.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"
#include "Calibration.h"
#include "SensorsADC.h"
#include <EEPROM.h>
#include "CanCmdBus.h"
#include <Main.h>

// -------------------- Debug UART (PA2 TX-only) --------------------
HardwareSerial Debug(PA3, PA2); // RX=PA3 (unused), TX=PA2

// -------------------- Hardware wiring --------------------
// Encoder (uses PB7=SDA, PA15=SCL below)
EncoderAS5600 encoder;

// Step gen: TIM1 CH2 → PB14 (STEP), PB15 (DIR), PB5 (EN active-low)
StepperControl stepgen(PB14, PB15, PB5, 200);

// TMC2209 (UART): RX=PA10, TX=PB6
#define R_SENSE 0.11f
HardwareSerial TMCSerial(PA10, PB6);
TMC2209Stepper driver(&TMCSerial, R_SENSE, 0b00);

// Axis controller
AxisController axis(encoder, stepgen, driver, 200, 256);

// External STEP/DIR/EN (moved DIR off PA2 to avoid conflict with Debug TX)
AxisController::ExtPins extPins{
    /*step*/ PA0, /*dir*/ PA1, /*en*/ PA4, /*enActiveLow*/ true};

// -------------------- Config storage --------------------
ConfigStore cfgStore;
AxisConfig cfg;

// -------------------- ADC Sensors --------------------
SensorsADC sensors;

// -------------------- 20 kHz control timer --------------------
HardwareTimer Tim6(TIM6);
static constexpr float CTRL_HZ = 20000.0f;
static float DT_SEC = 1.0f / CTRL_HZ;
static unsigned long lastms = 0; // Declare lastms as an unsigned long

void setup()
{
  // Start debug serial
  Debug.begin(115200);

  // Begin EEPROM
  EEPROM.begin();

  // ADC sensors
  sensors.begin();

  cfgStore.defaults(cfg);
  CanCmd_SetCallbacks({
      .setTargetAngleRad = [](double rad) { axis.setTargetAngleRad(rad); },
      .setMaxRps = [](float rps) { axis.setLimits(rps); },
      .setPid = [](float kp, float ki, float kd) { axis.setPID(kp, ki, kd); },
      .setCurrentLimit = [](float amps) { driver.rms_current(amps * 1000.0f); },
  });
  CanCmd_Begin();

  // Encoder on PB7 (SDA), PA15 (SCL) — update to your actual wiring
  if (!encoder.begin(PB7, PA15))
  {
    while (1)
    {
      Debug.println("[ERR] AS5600 not found");
      delay(500);
    }
  }
  encoder.setVelAlpha(0.75f); // mild smoothing
  encoder.setInvert(cfg.encInvert);

  // Bring up axis + driver
  TMCSerial.begin(115200);
  axis.begin();

  AxisController::TmcConfig tmcCfg;
  tmcCfg.mA = cfg.driver_mA;
  tmcCfg.toff = 5;
  tmcCfg.blank = 24;
  tmcCfg.spreadAlways = !cfg.stealthChop;
  tmcCfg.stealth = cfg.stealthChop;
  tmcCfg.spreadSwitchRPS = 6.0;
  axis.configureDriver(tmcCfg);

  axis.setPID(cfg.Kp, cfg.Ki, cfg.Kd);
  axis.setLimits(cfg.maxRPS);
  axis.setMicrosteps(cfg.microsteps);

  axis.attachExternal(extPins);
  axis.setExternalMode(cfg.externalMode);

  axis.setTargetAngleRad(0);
}

void loop()
{
  DT_SEC = (millis() - lastms) * 0.001f;
  lastms = millis();
  // 1) Run axis PID + stepgen
  axis.update(DT_SEC);

  sensors.update(); // ADC sensors (non-blocking)
  CanCmd_Poll();
}