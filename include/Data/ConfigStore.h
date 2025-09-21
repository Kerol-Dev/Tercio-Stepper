#pragma once
#include <Arduino.h>
#include <stddef.h>

// Persisted axis configuration
struct AxisConfig
{
  // identity & integrity
  uint32_t magic = 0xA1A1C0DE; // bump if layout changes
  uint16_t version = 2;        // bump when fields change
  uint16_t length = sizeof(AxisConfig);
  uint32_t crc32 = 0;

  // core motion
  uint16_t microsteps = 256;
  uint16_t units = 0;
  bool encInvert = false;
  bool dirInvert = false;
  bool stealthChop = true;
  bool externalMode = false;

  // encoder zero (raw AS5600 counts)
  int32_t encZeroCounts = 0;

  // driver / motion limits
  uint16_t driver_mA = 800;
  double maxRPS = 5.0;

  // PID (pos->vel)
  double Kp = 3.0;
  double Ki = 0.0;
  double Kd = 0.0;

  // reserved
  uint32_t flags = 0;

  uint32_t canArbId = 0x001;  // our arbitration ID on the bus
  bool canIdExtended = false; // 11-bit (false) or 29-bit (true)
  uint16_t nodeId16 = 0x0042; // in-frame ID16
};

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