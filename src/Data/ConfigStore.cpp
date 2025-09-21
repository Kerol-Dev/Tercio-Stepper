#include "ConfigStore.h"
#include <EEPROM.h>

namespace {
  // CRC32 (poly 0xEDB88320)
  uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    crc = ~crc;
    while (len--) {
      crc ^= *data++;
      for (int i = 0; i < 8; ++i) {
        crc = (crc >> 1) ^ (0xEDB88320UL & (-(int)(crc & 1)));
      }
    }
    return ~crc;
  }

  constexpr uint32_t kMagic = 0xA1A1C0DE;
}

ConfigStore::ConfigStore(size_t eeprom_base)
: _base(eeprom_base) {}

void ConfigStore::defaults(AxisConfig& c) const {
  c = AxisConfig{};
}

bool ConfigStore::load(AxisConfig& c) const {
  AxisConfig tmp{};
  if (!eepromRead(0, &tmp, sizeof(tmp))) return false;

  const uint32_t saved = tmp.crc32;
  tmp.crc32 = 0;
  const uint32_t calc = crc32_update(0, reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
  if (calc != saved) return false;

  c = tmp;
  return true;
}

bool ConfigStore::save(AxisConfig& c) const {
  c.crc32  = 0;
  const uint32_t calc = crc32_update(0, reinterpret_cast<const uint8_t*>(&c), sizeof(c));
  c.crc32 = calc;
  return eepromWrite(0, &c, sizeof(c));
}

bool ConfigStore::eepromRead(size_t off, void* buf, size_t len) const {
  if (_base + off + len > EEPROM.length()) return false;
  auto* p = static_cast<uint8_t*>(buf);
  for (size_t i = 0; i < len; ++i) {
    p[i] = EEPROM.read(_base + off + i);
  }
  return true;
}

bool ConfigStore::eepromWrite(size_t off, const void* buf, size_t len) const {
  if (_base + off + len > EEPROM.length()) return false;
  auto* p = static_cast<const uint8_t*>(buf);
  for (size_t i = 0; i < len; ++i) {
    EEPROM.update(_base + off + i, p[i]);
  }
  return true;
}