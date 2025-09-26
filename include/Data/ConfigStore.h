#pragma once
#include <Arduino.h>
#include <stddef.h>

// Natural C++ config with defaults
struct AxisConfig {
  uint32_t crc32 = 0;
  uint16_t microsteps = 16;
  uint16_t stepsPerRev = 200;
  uint8_t  units = 0;
  bool     encInvert = false;
  bool     dirInvert = false;
  bool     stealthChop = true;
  bool     externalMode = false;
  bool     minTriggered = false;
  bool     maxTriggered = false;
  bool     enableProtection = true;
  uint16_t encZeroCounts = 0;
  uint16_t driver_mA = 1000;
  float    maxRPS = 5.0f;
  float    Kp = 3.0f;
  float    Ki = 0.0f;
  float    Kd = 0.0f;
  uint16_t canArbId = 0x001;
};

// Packed wire version (no defaults!)
#pragma pack(push,1)
struct AxisConfigWire {
  uint32_t crc32;
  uint16_t microsteps;
  uint16_t stepsPerRev;
  uint8_t  units;
  uint8_t  flags;     // pack encInvert..externalMode into bits
  uint16_t encZeroCounts;
  uint16_t driver_mA;
  float    maxRPS;
  float    Kp, Ki, Kd;
  uint16_t canArbId;
};
#pragma pack(pop)

// Convert native → wire
inline AxisConfigWire toWire(const AxisConfig& cfg) {
  AxisConfigWire w{};
  w.crc32        = cfg.crc32;
  w.microsteps   = cfg.microsteps;
  w.stepsPerRev = cfg.stepsPerRev;
  w.units        = cfg.units;
  w.flags        = (cfg.encInvert?1:0) | (cfg.dirInvert?2:0) |
                   (cfg.stealthChop?4:0) | (cfg.externalMode?8:0) | (cfg.minTriggered?16:0) | (cfg.maxTriggered?32:0) | (cfg.enableProtection?64:0);
  w.encZeroCounts= cfg.encZeroCounts;
  w.driver_mA    = cfg.driver_mA;
  w.maxRPS       = cfg.maxRPS;
  w.Kp           = cfg.Kp;
  w.Ki           = cfg.Ki;
  w.Kd           = cfg.Kd;
  w.canArbId     = cfg.canArbId;
  return w;
}


class ConfigStore
{
public:
  explicit ConfigStore(size_t eeprom_base = 0);

  void defaults(AxisConfig &c) const;
  bool load(AxisConfig &c) const;
  bool save(AxisConfig &c) const;

private:
  bool eepromRead(size_t off, void *buf, size_t len) const;
  bool eepromWrite(size_t off, const void *buf, size_t len) const;

private:
  size_t _base{0};
};