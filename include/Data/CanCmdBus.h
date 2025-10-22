#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>

namespace CanCmdBus {

#ifndef CANCMD_MAX_HANDLERS
#define CANCMD_MAX_HANDLERS 32
#endif

// Parsed RX frame
struct CmdFrame {
  uint16_t id;            // 11-bit ID
  uint8_t  cmd;           // data[0]
  const uint8_t* payload; // data[1..]
  uint8_t  len;           // payload length (0..63)
};

using Handler = void (*)(const CmdFrame&);

// Init / poll
bool begin(uint32_t nominal_bps = 500000,
           uint8_t  data_factor  = 5,
           uint8_t  rxPin        = 0xFF,
           uint8_t  txPin        = 0xFF,
           bool     normalFD     = true);

void poll();

// Handlers / filters
bool registerHandler(uint8_t cmd, Handler fn);
void setIdFilter(uint16_t filter, uint16_t mask);

// TX helpers
void setDefaultTxId(uint16_t id);

bool send(uint16_t id, uint8_t cmd, const void* data, uint8_t len);
bool sendDefault(uint8_t cmd, const void* data, uint8_t len);

bool sendF32(uint16_t id, uint8_t cmd, float value);
bool sendF32(uint8_t cmd, float value);

bool sendI32(uint16_t id, uint8_t cmd, int32_t value);
bool sendI32(uint8_t cmd, int32_t value);

template <typename T>
bool sendStruct(uint16_t id, uint8_t cmd, const T& pod) {
  return send(id, cmd, &pod, (uint8_t)sizeof(T));
}
template <typename T>
bool sendStruct(uint8_t cmd, const T& pod) {
  return sendDefault(cmd, &pod, (uint8_t)sizeof(T));
}

// Little-endian readers
inline bool readU8(const uint8_t* b, size_t len, size_t off, uint8_t& out) {
  if (off + 1 > len) return false;
  out = b[off];
  return true;
}
inline bool readU16(const uint8_t* b, size_t len, size_t off, uint16_t& out) {
  if (off + 2 > len) return false;
  out = (uint16_t)b[off] | ((uint16_t)b[off + 1] << 8);
  return true;
}
inline bool readU32(const uint8_t* b, size_t len, size_t off, uint32_t& out) {
  if (off + 4 > len) return false;
  out = (uint32_t)b[off] |
        ((uint32_t)b[off + 1] << 8) |
        ((uint32_t)b[off + 2] << 16) |
        ((uint32_t)b[off + 3] << 24);
  return true;
}
inline bool readF32(const uint8_t* b, size_t len, size_t off, float& out) {
  if (off + 4 > len) return false;
  uint32_t v;
  if (!readU32(b, len, off, v)) return false;
  std::memcpy(&out, &v, 4);
  return true;
}

}