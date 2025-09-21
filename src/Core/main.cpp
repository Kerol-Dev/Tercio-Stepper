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
#include <ACANFD_STM32_Settings.h>
#include "CanCmdBus.h"
#include <Main.h>

enum : uint8_t
{
  CMD_TARGET_ANGLE = 0x01,
  CMD_SET_CURRENT_MA = 0x02,
  CMD_SET_SPEED_LIMIT = 0x03,
  CMD_SET_PID = 0x04,
  CMD_SET_ID = 0x05,
  CMD_SET_MICROSTEPS = 0x06,
  CMD_SET_STEALTHCHOP = 0x07,
  CMD_SET_EXT_MODE = 0x08,
  CMD_SET_UNITS = 0x09,
  CMD_SET_ENC_INVERT = 0x0A
};

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

static void setTarget(const CanCmdBus::CmdFrame &f)
{
  float angle;
  if (CanCmdBus::readF32(f.payload, f.len, 0, angle))
  {
    float adjustedAngle = angle;
    switch (cfg.units)
    {
    case 0: // degrees
      adjustedAngle = angle * DEG_TO_RAD;
      break;
    case 1: // radians
      adjustedAngle = angle;
      break;

    default:
      break;
    }
    axis.setTargetAngleRad(adjustedAngle);
  }
}

static void setCurrentMA(const CanCmdBus::CmdFrame &f)
{
  uint32_t ma;
  if (CanCmdBus::readU32(f.payload, f.len, 0, ma))
  {
    cfg.driver_mA = ma;
    driver.rms_current(ma);
    save(cfg);
  }
}

static void setSpeedLimit(const CanCmdBus::CmdFrame &f)
{
  float rps;
  if (CanCmdBus::readF32(f.payload, f.len, 0, rps))
  {
    cfg.maxRPS = rps;
    axis.setLimits(rps);
    save(cfg);
  }
}

static void setPID(const CanCmdBus::CmdFrame &f)
{
  float Kp, Ki, Kd;
  if (CanCmdBus::readF32(f.payload, f.len, 0, Kp) &&
      CanCmdBus::readF32(f.payload, f.len, 4, Ki) &&
      CanCmdBus::readF32(f.payload, f.len, 8, Kd))
  {
    cfg.Kp = Kp;
    cfg.Ki = Ki;
    cfg.Kd = Kd;
    axis.setPID(Kp, Ki, Kd);
    save(cfg);
  }
}

static void setID(const CanCmdBus::CmdFrame &f)
{
  uint16_t id;
  if (CanCmdBus::readU16(f.payload, f.len, 0, id))
  {
    cfg.canArbId = id;
    CanCmdBus::setIdFilter(cfg.canArbId, 0x7FF);
    save(cfg);
  }
}

static void setMicrosteps(const CanCmdBus::CmdFrame &f)
{
  uint16_t ms;
  if (CanCmdBus::readU16(f.payload, f.len, 0, ms))
  {
    cfg.microsteps = ms;
    axis.setMicrosteps(ms);
    save(cfg);
  }
}

static void setStealthChop(const CanCmdBus::CmdFrame &f)
{
  uint8_t sc;
  if (CanCmdBus::readU8(f.payload, f.len, 0, sc))
  {
    cfg.stealthChop = (sc != 0);
    AxisController::TmcConfig tmcCfg;
    tmcCfg.mA = cfg.driver_mA;
    tmcCfg.toff = 5;
    tmcCfg.blank = 24;
    tmcCfg.spreadAlways = !cfg.stealthChop;
    tmcCfg.stealth = cfg.stealthChop;
    tmcCfg.spreadSwitchRPS = 6.0;
    axis.configureDriver(tmcCfg);
    save(cfg);
  }
}

static void setExtMode(const CanCmdBus::CmdFrame &f)
{
  uint8_t em;
  if (CanCmdBus::readU8(f.payload, f.len, 0, em))
  {
    cfg.externalMode = (em != 0);
    axis.setExternalMode(cfg.externalMode);
    save(cfg);
  }
}

static void setUnits(const CanCmdBus::CmdFrame &f)
{
  uint16_t u;
  if (CanCmdBus::readU16(f.payload, f.len, 0, u))
  {
    cfg.units = u;
    save(cfg);
  }
}

static void setEncInvert(const CanCmdBus::CmdFrame &f)
{
  uint8_t ei;
  if (CanCmdBus::readU8(f.payload, f.len, 0, ei))
  {
    cfg.encInvert = (ei != 0);
    encoder.setInvert(cfg.encInvert);
    save(cfg);
  }
}

// Function to send data as a CAN message
static void sendData()
{
  // Define a struct to hold the data
  struct DataPacket
  {
    double currentSpeed;
    double currentAngle;
    double targetAngle;
    AxisConfig config; // Include everything from cfg
  };

  // Populate the struct with current values
  DataPacket packet;
  packet.currentSpeed = encoder.velocity(cfg.units ? EncoderAS5600::Degrees : EncoderAS5600::Radians);
  packet.currentAngle = encoder.angle(cfg.units ? EncoderAS5600::Degrees : EncoderAS5600::Radians);    // Assuming degrees
  packet.targetAngle = axis.targetAngleRad() * RAD_TO_DEG;        // Convert radians to degrees
  packet.config = cfg;

  // Send the struct as a CAN message
  if (!CanCmdBus::sendStruct(0x000, 0x01, packet)) // Example ID and CMD
  {
    Debug.println("Failed to send CAN message");
  }
}

void setup()
{
  // Start debug serial
  Debug.begin(115200);

  // Begin EEPROM
  EEPROM.begin();

  // ADC sensors
  sensors.begin();

  cfgStore.defaults(cfg);
  CanCmdBus::begin(500000, 5, PA11, PA12, true);
  CanCmdBus::setDebug(&Debug);
  CanCmdBus::setIdFilter(cfg.canArbId, 0x7FF);

  CanCmdBus::registerHandler(CMD_TARGET_ANGLE, setTarget);
  CanCmdBus::registerHandler(CMD_SET_CURRENT_MA, setCurrentMA);
  CanCmdBus::registerHandler(CMD_SET_SPEED_LIMIT, setSpeedLimit);
  CanCmdBus::registerHandler(CMD_SET_PID, setPID);
  CanCmdBus::registerHandler(CMD_SET_ID, setID);
  CanCmdBus::registerHandler(CMD_SET_MICROSTEPS, setMicrosteps);
  CanCmdBus::registerHandler(CMD_SET_STEALTHCHOP, setStealthChop);
  CanCmdBus::registerHandler(CMD_SET_EXT_MODE, setExtMode);
  CanCmdBus::registerHandler(CMD_SET_UNITS, setUnits);
  CanCmdBus::registerHandler(CMD_SET_ENC_INVERT, setEncInvert);
  
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

  axis.update(DT_SEC);

  sensors.update();
  if (millis() % 100 == 0) // every 100 ms
  {
    sendData();
  }
  CanCmdBus::poll();
}