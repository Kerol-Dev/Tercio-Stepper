// -----------------------------------------------------------------------------
// Main firmware
// -----------------------------------------------------------------------------
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
#include "StepperHoming.h"
#include "Main.h"

// -----------------------------------------------------------------------------
// Developer mode (compile-time)
// Set to 0 for production builds to avoid creating a hardware UART for debug.
// -----------------------------------------------------------------------------
#ifndef DEVELOPER_MODE
#define DEVELOPER_MODE 1
#endif
constexpr bool kDeveloperMode = (DEVELOPER_MODE != 0);

// Debug sink
#if DEVELOPER_MODE
// Debug UART (PA2 TX-only). RX=PA3 (unused), TX=PA2
HardwareSerial Debug(PA3, PA2);
#else
// Minimal no-op stream compatible with Print-like interfaces used by the firmware.
class NullStream : public Print
{
public:
  void begin(unsigned long) {}
  void println(const char *) {}
  void printf(const char *, ...) {}
  size_t write(uint8_t) override { return 1; }
  using Print::write;
};
static NullStream Debug;
#endif

// Debug macros
#if DEVELOPER_MODE
#define DBG_INIT(baud) \
  do                   \
  {                    \
    Debug.begin(baud); \
  } while (0)
#define DBG_PRINTLN(msg) \
  do                     \
  {                      \
    Debug.println(msg);  \
  } while (0)
#define DBG_PRINTF(...)        \
  do                           \
  {                            \
    Debug.printf(__VA_ARGS__); \
  } while (0)
#else
#define DBG_INIT(baud) \
  do                   \
  {                    \
  } while (0)
#define DBG_PRINTLN(msg) \
  do                     \
  {                      \
  } while (0)
#define DBG_PRINTF(...) \
  do                    \
  {                     \
  } while (0)
#endif

// -----------------------------------------------------------------------------
// Commands
// -----------------------------------------------------------------------------
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
  CMD_SET_ENC_INVERT = 0x0A,
  CMD_SET_ENABLED = 0x0B,
  CMD_SET_STEPS_PER_REV = 0x0C,
  CMD_DO_CALIBRATE = 0x0D,
  CMD_DO_HOMING = 0x0E,
  CMD_SET_ENDSTOP = 0x0F
};

// -----------------------------------------------------------------------------
// Hardware pins / instances
// -----------------------------------------------------------------------------
static constexpr uint8_t PIN_STEP = PB14;
static constexpr uint8_t PIN_DIR = PB15;
static constexpr uint8_t PIN_EN = PB5; // EN active-low
static constexpr uint8_t PIN_I2C_SDA = PB7;
static constexpr uint8_t PIN_I2C_SCL = PA15;
static constexpr uint8_t PIN_HOM_IN1 = PB12;
static constexpr uint8_t PIN_HOM_IN2 = PB13;

EncoderAS5600 encoder;
StepperHoming homing;
StepperControl stepgen(PIN_STEP, PIN_DIR, PIN_EN, 200);

#define R_SENSE 0.11f
HardwareSerial TMCSerial(PA10, PB6); // TMC2209 UART: RX=PA10, TX=PB6
TMC2209Stepper driver(&TMCSerial, R_SENSE, 0b00);

AxisController axis(encoder, stepgen, driver, 200, 16);

AxisController::ExtPins extPins{
    /*step*/ PA0, /*dir*/ PA1, /*en*/ PA4, /*enActiveLow*/ true};

// -----------------------------------------------------------------------------
// Config / sensors / timing
// -----------------------------------------------------------------------------
ConfigStore cfgStore;
AxisConfig cfg;
SensorsADC sensors;

static float g_dtSec = 0.f;
static unsigned long g_lastMs = 0;
static float limitAngleCovered = 0.0f;

// -----------------------------------------------------------------------------
// Homing wire payload <B f B f B> (bools as u8; offset/speed as float32)
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
struct HomingWire
{
  uint8_t useIN1Trigger; // 0/1
  float offset;          // device units (deg/rad per cfg.units)
  uint8_t activeLow;     // 0/1
  float speed;           // rps (or chosen units)
  uint8_t direction;     // 0=negative, 1=positive
};
#pragma pack(pop)
static_assert(sizeof(HomingWire) == 11, "HomingWire must be 11 bytes");

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static inline bool readBool01(const uint8_t *p, uint8_t len, uint8_t off, bool &out)
{
  uint8_t v;
  if (!CanCmdBus::readU8(p, len, off, v))
    return false;
  out = (v != 0);
  return true;
}

// -----------------------------------------------------------------------------
// Handlers
// -----------------------------------------------------------------------------
static void onTarget(const CanCmdBus::CmdFrame &f)
{
  float angle;
  if (!CanCmdBus::readF32(f.payload, f.len, 0, angle))
    return;

  const float rad = (cfg.units == 1) ? angle * DEG_TO_RAD : angle;
  axis.setTargetAngleRad(rad);
}

static void onCurrentMA(const CanCmdBus::CmdFrame &f)
{
  uint16_t ma;
  if (!CanCmdBus::readU16(f.payload, f.len, 0, ma))
    return;

  ma = (ma < 50) ? 50 : (ma > 2000 ? 2000 : ma);
  cfg.driver_mA = ma;
  driver.rms_current(ma);
  cfgStore.save(cfg);
}

static void onHoming(const CanCmdBus::CmdFrame &f)
{
  stepgen.enable();

  if (f.len != sizeof(HomingWire))
  {
    DBG_PRINTF("[HOM] Bad payload len=%u (expected %u)\r\n",
               unsigned(f.len), unsigned(sizeof(HomingWire)));
    return;
  }

  HomingWire hw{};
  memcpy(&hw, f.payload, sizeof(HomingWire));

  HomingConfig hcfg{
      static_cast<uint8_t>(PIN_HOM_IN1), // inMinPin
      static_cast<uint8_t>(PIN_HOM_IN2), // inMaxPin
      (hw.activeLow != 0),               // minActiveLow
      (hw.activeLow != 0),               // maxActiveLow
      hw.speed,                          // seekSpeed (sign enforced in setVel below)
      30000u,                            // timeoutMs
      hw.offset                          // backoffOffset
  };
  homing.begin(hcfg);

  const bool useIN1 = (hw.useIN1Trigger != 0);
  const bool dirPos = (hw.direction != 0);

  const bool ok = homing.home(
      // setVel
      [&](float v)
      {
        const float sp = dirPos ? fabsf(v) : -fabsf(v);
        stepgen.setSpeedRPS(sp);
      },
      // stop
      [&]()
      { stepgen.stop(); },
      // encoder
      encoder,
      // setZero
      [&](float /*z*/)
      {
        encoder.calibrateZero();
      },
      // seekToMin
      useIN1);

  DBG_PRINTLN(ok ? "[HOM] Done" : "[HOM] FAILED");
}

static void onStepsPerRev(const CanCmdBus::CmdFrame &f)
{
  uint16_t rev;
  if (!CanCmdBus::readU16(f.payload, f.len, 0, rev))
    return;

  cfg.stepsPerRev = rev;
  axis.setFullSteps(rev);
  stepgen.setFullSteps(rev);
  cfgStore.save(cfg);
}

static void onSpeedLimit(const CanCmdBus::CmdFrame &f)
{
  float rps;
  if (!CanCmdBus::readF32(f.payload, f.len, 0, rps))
    return;

  rps = (rps < 0.f) ? 0.f : (rps > 200.f ? 200.f : rps);
  cfg.maxRPS = rps;
  axis.setLimits(rps);
  cfgStore.save(cfg);
}

static void onPID(const CanCmdBus::CmdFrame &f)
{
  float Kp, Ki, Kd;
  if (!CanCmdBus::readF32(f.payload, f.len, 0, Kp))
    return;
  if (!CanCmdBus::readF32(f.payload, f.len, 4, Ki))
    return;
  if (!CanCmdBus::readF32(f.payload, f.len, 8, Kd))
    return;

  cfg.Kp = Kp;
  cfg.Ki = Ki;
  cfg.Kd = Kd;
  axis.setPID(Kp, Ki, Kd);
  cfgStore.save(cfg);
}

static void onID(const CanCmdBus::CmdFrame &f)
{
  uint16_t id;
  if (!CanCmdBus::readU16(f.payload, f.len, 0, id))
    return;

  id &= 0x7FF;
  cfg.canArbId = id;
  CanCmdBus::setIdFilter(cfg.canArbId, 0x7FF);
  cfgStore.save(cfg);
}

static void onMicrosteps(const CanCmdBus::CmdFrame &f)
{
  uint16_t ms;
  if (!CanCmdBus::readU16(f.payload, f.len, 0, ms))
    return;

  if (ms == 0)
    ms = 1;
  cfg.microsteps = ms;
  axis.setMicrosteps(ms);
  cfgStore.save(cfg);
}

static void onStealthChop(const CanCmdBus::CmdFrame &f)
{
  bool sc;
  if (!readBool01(f.payload, f.len, 0, sc))
    return;

  cfg.stealthChop = sc;

  AxisController::TmcConfig tmc{};
  tmc.mA = cfg.driver_mA;
  tmc.toff = 5;
  tmc.blank = 24;
  tmc.spreadAlways = !cfg.stealthChop;
  tmc.stealth = cfg.stealthChop;
  tmc.spreadSwitchRPS = 6.0;
  axis.configureDriver(tmc);

  cfgStore.save(cfg);
}

static void onEnabled(const CanCmdBus::CmdFrame &f)
{
  bool en;
  if (!readBool01(f.payload, f.len, 0, en))
    return;

  if (en)
  {
    stepgen.enable();
  }
  else
  {
    stepgen.stop();
    stepgen.disable();
  }
}

static void onCalibrate(const CanCmdBus::CmdFrame &f)
{
  (void)f;
  Calibrate_EncoderDirection(encoder, stepgen, axis, cfg, Debug, 1.0, 2000);
  cfgStore.save(cfg);
}

static void enableEndstop(const CanCmdBus::CmdFrame &f)
{
  bool ep;
  if (!readBool01(f.payload, f.len, 0, ep))
    return;

  cfg.enableProtection = ep;
  cfgStore.save(cfg);
}

static void onExtMode(const CanCmdBus::CmdFrame &f)
{
  bool em;
  if (!readBool01(f.payload, f.len, 0, em))
    return;

  cfg.externalMode = em;
  axis.setExternalMode(cfg.externalMode);
  cfgStore.save(cfg);
}

static void onUnits(const CanCmdBus::CmdFrame &f)
{
  uint8_t u;
  if (!CanCmdBus::readU8(f.payload, f.len, 0, u))
    return;

  cfg.units = (u != 0) ? 1 : 0; // 0=radians, 1=degrees
  cfgStore.save(cfg);
}

static void onEncInvert(const CanCmdBus::CmdFrame &f)
{
  bool ei;
  if (!readBool01(f.payload, f.len, 0, ei))
    return;

  cfg.encInvert = ei;
  encoder.setInvert(cfg.encInvert);
  cfgStore.save(cfg);
}

// -----------------------------------------------------------------------------
// Telemetry (CAN broadcast)
// -----------------------------------------------------------------------------
static void sendData()
{
  struct DataPacket
  {
    AxisConfigWire config;
    float currentSpeed;
    float currentAngle;
    float targetAngle;
    float temperature;
  };

  DataPacket pkt;
  const bool useDeg = (cfg.units == 1);

  pkt.currentSpeed = encoder.velocity(useDeg ? EncoderAS5600::Degrees
                                             : EncoderAS5600::Radians);
  pkt.currentAngle = encoder.angle(useDeg ? EncoderAS5600::Degrees
                                          : EncoderAS5600::Radians);
  pkt.targetAngle = axis.targetAngleRad() * (useDeg ? RAD_TO_DEG : 1.0);
  pkt.temperature = sensors.temperatureC();
  pkt.config = toWire(cfg);

  if (!CanCmdBus::sendStruct(0x000, 0x01, pkt))
  {
    DBG_PRINTLN("Failed to send CAN message");
  }
}

bool overTemperatureProtection()
{
  return sensors.temperatureC() >= 95.0f;
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup()
{
  DBG_INIT(115200);
  EEPROM.begin();
  sensors.begin();

  if (!cfgStore.load(cfg))
  {
    DBG_PRINTLN("[WARN] Config load failed, using defaults");
    cfgStore.defaults(cfg);
    cfgStore.save(cfg);
  }
  else
  {
    DBG_PRINTLN("[INFO] Config loaded");
  }

  // CAN
  CanCmdBus::begin(500000, 5, PA11, PA12, true);
#if DEVELOPER_MODE
  CanCmdBus::setDebug(&Debug);
#endif
  CanCmdBus::setIdFilter(cfg.canArbId, 0x7FF);

  // Command handlers
  CanCmdBus::registerHandler(CMD_TARGET_ANGLE, onTarget);
  CanCmdBus::registerHandler(CMD_SET_CURRENT_MA, onCurrentMA);
  CanCmdBus::registerHandler(CMD_SET_SPEED_LIMIT, onSpeedLimit);
  CanCmdBus::registerHandler(CMD_SET_PID, onPID);
  CanCmdBus::registerHandler(CMD_SET_ID, onID);
  CanCmdBus::registerHandler(CMD_SET_MICROSTEPS, onMicrosteps);
  CanCmdBus::registerHandler(CMD_SET_STEALTHCHOP, onStealthChop);
  CanCmdBus::registerHandler(CMD_SET_EXT_MODE, onExtMode);
  CanCmdBus::registerHandler(CMD_SET_UNITS, onUnits);
  CanCmdBus::registerHandler(CMD_SET_ENC_INVERT, onEncInvert);
  CanCmdBus::registerHandler(CMD_SET_ENABLED, onEnabled);
  CanCmdBus::registerHandler(CMD_SET_STEPS_PER_REV, onStepsPerRev);
  CanCmdBus::registerHandler(CMD_DO_CALIBRATE, onCalibrate);
  CanCmdBus::registerHandler(CMD_DO_HOMING, onHoming);
  CanCmdBus::registerHandler(CMD_SET_ENDSTOP, enableEndStop);

  // Encoder
  if (!encoder.begin(PIN_I2C_SDA, PIN_I2C_SCL))
  {
    while (1)
    {
      DBG_PRINTLN("[ERR] AS5600 not found");
      delay(500);
    }
  }
  encoder.setVelAlpha(1); // mild smoothing
  encoder.setInvert(cfg.encInvert);

  // Axis + driver
  TMCSerial.begin(115200);
  axis.begin();

  AxisController::TmcConfig tmc{};
  tmc.mA = cfg.driver_mA;
  tmc.toff = 5;
  tmc.blank = 24;
  tmc.spreadAlways = !cfg.stealthChop;
  tmc.stealth = cfg.stealthChop;
  tmc.spreadSwitchRPS = 6.0;
  axis.configureDriver(tmc);

  stepgen.setFullSteps(cfg.stepsPerRev);
  axis.setFullSteps(cfg.stepsPerRev);
  axis.setPID(cfg.Kp, cfg.Ki, cfg.Kd);
  axis.setLimits(cfg.maxRPS);
  axis.setMicrosteps(cfg.microsteps);

  axis.attachExternal(extPins);
  axis.setExternalMode(cfg.externalMode);
  axis.setTargetAngleRad(0);
}

void loop()
{
  const unsigned long now = millis();
  g_dtSec = (now - g_lastMs) * 0.001f;
  g_lastMs = now;

  axis.update(g_dtSec);
  sensors.update();
  homing.update();

  cfg.minTriggered = homing.minTriggered();
  cfg.maxTriggered = homing.maxTriggered();

  if (cfg.enableEndStop)
  {
    if (homing.minTriggered())
    {
      if (limitAngleCovered == 0)
      {
        limitAngleCovered = encoder.angle();
      }
      if (axis.targetAngleRad() < limitAngleCovered)
      {
        axis.setTargetAngleRad(limitAngleCovered);
      }
    }
    else if (homing.maxTriggered())
    {
      if (limitAngleCovered == 0)
      {
        limitAngleCovered = encoder.angle();
      }
      if (axis.targetAngleRad() > limitAngleCovered)
      {
        axis.setTargetAngleRad(limitAngleCovered);
      }
    }
    else
    {
      limitAngleCovered = 0.0f;
    }
  }

  if (overTemperatureProtection())
  {
    stepgen.stop();
  }

  if ((now % 50) == 0)
  { // ~20 Hz gate
    sendData();
  }
  CanCmdBus::poll();
}