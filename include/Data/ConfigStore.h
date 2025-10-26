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
  bool     enableEndStop = false;
  bool     externalEncoder = false;
  bool     externalSPI = false;
  bool     calibratedOnce = false;
  uint16_t encZeroCounts = 0;
  uint16_t driver_mA = 1200;
  float    maxRPS = 25.0f;
  float    maxRPS2 = 200.0f;
  float    Kp = 5.0f;
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
  uint8_t  flags;
  uint16_t encZeroCounts;
  uint16_t driver_mA;
  float    maxRPS;
  float    maxRPS2;
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
                   (cfg.stealthChop?4:0) | (cfg.externalMode?8:0) | (cfg.enableEndStop?16:0) | (cfg.externalEncoder?32:0) | (cfg.calibratedOnce?64:0);
  w.encZeroCounts= cfg.encZeroCounts;
  w.driver_mA    = cfg.driver_mA;
  w.maxRPS       = cfg.maxRPS;
  w.maxRPS2      = cfg.maxRPS2;
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