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
#include <CanFD_HAL.h>
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

// -------------------- Target toggling --------------------
static const double TGT_RIGHT_RAD = +360.0 * DEG_TO_RAD;
static const double TGT_LEFT_RAD = -360.0 * DEG_TO_RAD;
static const double REACH_TOL_RAD = 0.01 * DEG_TO_RAD; // switch when within ±3°
static volatile bool goRight = true;

CanHAL::CanIds canCfg;

// -------------------- Telemetry ring buffer (captured in ISR) --------------------
struct Telemetry
{
  float pos_deg;
  float tgt_deg;
  float vel_deg_s;
  float rps;
  float vbus_V;
  float temp_C;
  uint16_t micro;
  uint8_t mode_ext;
  uint8_t mag_bits;
};
static constexpr uint16_t LOG_CAP = 256; // small lock-free ring
volatile uint16_t logHead = 0, logTail = 0;
Telemetry logBuf[LOG_CAP];

// Push (ISR-safe, lock-free single producer / single consumer)
inline void logPush(const Telemetry &s)
{
  uint16_t h = logHead;
  uint16_t n = (uint16_t)((h + 1) & (LOG_CAP - 1));
  if (n == logTail)
    return; // drop if full
  logBuf[h] = s;
  logHead = n;
}

// Pop (main context)
bool logPop(Telemetry &out)
{
  uint16_t t = logTail;
  if (t == logHead)
    return false;
  out = logBuf[t];
  logTail = (uint16_t)((t + 1) & (LOG_CAP - 1));
  return true;
}

uint32_t lastms = 0;

void setup()
{
  // Start debug serial
  Debug.begin(115200);

  // Begin EEPROM
  EEPROM.begin();

  // ADC sensors
  sensors.begin();

  // Load config or defaults
  // if (!cfgStore.load(cfg)) {
  //   Debug.println("[CFG] No valid config; using defaults.");
  //   cfgStore.defaults(cfg);
  //   cfg.microsteps = 256;           // your current default
  //   cfg.driver_mA  = 800;
  //   cfg.maxRPS     = 6.0;
  //   cfg.maxAccelRPS2 = 80.0;
  //   cfg.Kp = 3.0; cfg.Ki = 0.0; cfg.Kd = 0.0;
  //   cfg.spreadSwitchRPS = 4.0;
  //   cfgStore.save(cfg);
  // } else {
  //   Debug.println("[CFG] Loaded config from EEPROM.");
  // }
  cfgStore.defaults(cfg);
  canCfg.arbId = cfg.canArbId;
  canCfg.isExtended = cfg.canIdExtended;
  canCfg.nodeId16 = cfg.nodeId16;

  if (!CanHAL::begin(1'000'000, 2'000'000, canCfg))
  {
    Debug.println("[CAN] init failed");
  }

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
  goRight = false; // so next flip goes left

  // Calibrate_EncoderDirection(encoder, stepgen, axis, cfg, Debug, 1, 2000);
}

void loop()
{
  DT_SEC = (millis() - lastms) * 0.001f;
  lastms = millis();
  // 1) Run axis PID + stepgen
  axis.update(DT_SEC);

  sensors.update(); // ADC sensors (non-blocking)

  // 2) Auto-toggle target at endpoints
  const double pos = encoder.angle(EncoderAS5600::Radians);
  const double tgt = axis.targetAngleRad();
  const double err = fabs(tgt - pos);
  if (err < REACH_TOL_RAD)
  {
    goRight = !goRight;
    axis.setTargetAngleRad(goRight ? TGT_RIGHT_RAD : TGT_LEFT_RAD);
  }
  Telemetry s;
  s.pos_deg = (float)(pos * RAD_TO_DEG);
  s.tgt_deg = (float)(axis.targetAngleRad() * RAD_TO_DEG);
  const double vel_deg_s = encoder.velocity(EncoderAS5600::Degrees);
  s.vel_deg_s = (float)vel_deg_s;
  s.rps = (float)(vel_deg_s / 360.0);
  s.vbus_V = sensors.vbus_V();
  s.temp_C = sensors.temperatureC();
  s.micro = stepgen.microstep();
  s.mode_ext = axis.externalMode() ? 1 : 0;
  uint8_t m = 0;
  if (encoder.magnetPresent())
    m |= 1u << 0;
  if (encoder.magnetTooWeak())
    m |= 1u << 1;
  if (encoder.magnetTooStrong())
    m |= 1u << 2;
  s.mag_bits = m;

  Debug.print("pos=");
  Debug.print(s.pos_deg, 2);
  Debug.print("°  tgt=");
  Debug.print(s.tgt_deg, 2);
  Debug.print("°  vel=");
  Debug.print(s.vel_deg_s, 1);
  Debug.print("°/s  rps=");
  Debug.print(s.rps, 3);
  Debug.print("  VBUS=");
  Debug.print(s.vbus_V, 2);
  Debug.print("V  TEMP=");
  Debug.print(s.temp_C, 1);
  Debug.print("C  µ=");
  Debug.print(s.micro);
  Debug.print("  mode=");
  Debug.print(s.mode_ext ? "EXT" : "INT");
  Debug.print("  MAG[");
  Debug.print((s.mag_bits & 0x01) ? "P" : "!");
  Debug.print((s.mag_bits & 0x02) ? "W" : "-");
  Debug.print((s.mag_bits & 0x04) ? "H" : "-");
  Debug.println("]");
}