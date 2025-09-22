#pragma once
#include <stdint.h>
#include <stddef.h>

class Print;

namespace CanCmdBus {

// ---------------------- Tuning ----------------------
#ifndef CANCMD_MAX_HANDLERS
#define CANCMD_MAX_HANDLERS 32
#endif

// ---------------------- Types -----------------------
// A parsed command frame from CAN bus (RX)
struct CmdFrame {
  uint16_t id;            // standard ID (11-bit), masked with current filter
  uint8_t  cmd;           // byte 0 of CAN data
  const uint8_t* payload; // pointer to bytes after cmd (CAN data[1..])
  uint8_t  len;           // payload length (0..63 for FD; your bridge uses <=32)
};

// Each command handler gets the parsed frame
using Handler = void (*)(const CmdFrame&);

// ------------------ Public API (init / poll) ----------------------
/**
 * @brief Initialize FDCAN1 and the command bus.
 * @param nominal_bps   Nominal (arbitration) bit rate (default 500000).
 * @param data_factor   Data phase factor (x2..x16). Use 5 for ~2.5 Mbps.
 * @param rxPin         FDCAN1 RX pin (default PA11 if 0xFF).
 * @param txPin         FDCAN1 TX pin (default PA12 if 0xFF).
 * @param normalFD      true = NORMAL_FD (real bus), false = EXTERNAL_LOOP_BACK.
 * @return true on success (FDCAN started), false otherwise.
 */
bool begin(uint32_t nominal_bps = 500000,
           uint8_t  data_factor  = 5,
           uint8_t  rxPin        = 0xFF,
           uint8_t  txPin        = 0xFF,
           bool     normalFD     = true);

/** @brief Poll CAN RX FIFO and dispatch commands. Call this frequently from loop(). */
void poll();

// ------------------ Public API (handlers / filters) ----------------------
/** @brief Register a command handler for a specific CMD byte. */
bool registerHandler(uint8_t cmd, Handler fn);

/** @brief Set a standard ID filter/mask. Passes if (id & mask) == (filter & mask).
 *         Defaults: filter=0x000, mask=0x000 → accept all. */
void setIdFilter(uint16_t filter, uint16_t mask);

/** @brief Optional: set a Print* for debug logs (e.g., &Serial). Pass nullptr to disable logs. */
void setDebug(Print* dbg);

// ------------------ Public API (TX helpers) ----------------------
/** @brief Set a default TX ID (11-bit standard). Optional; used by sendDefault* helpers. */
void setDefaultTxId(uint16_t id);

/** @brief Send raw bytes as a command payload on CAN-FD.
 *  Data on the wire = [CMD][PAYLOAD...], DLC auto-mapped.
 *  @param id    11-bit standard ID (0..0x7FF)
 *  @param cmd   command byte
 *  @param data  pointer to payload bytes (may be nullptr if len=0)
 *  @param len   payload length (0..32 recommended to match your USB bridge)
 *  @return true on success (queued), false on failure.
 */
bool send(uint16_t id, uint8_t cmd, const void* data, uint8_t len);

/** @brief Send using the default TX ID set via setDefaultTxId. */
bool sendDefault(uint8_t cmd, const void* data, uint8_t len);

/** @brief Send a single float as payload (little-endian). */
bool sendF32(uint16_t id, uint8_t cmd, float value);
bool sendF32(uint8_t cmd, float value); // uses default TX ID

/** @brief Send a single int32 as payload (little-endian). */
bool sendI32(uint16_t id, uint8_t cmd, int32_t value);
bool sendI32(uint8_t cmd, int32_t value); // uses default TX ID

/** @brief Send any POD/struct by value (raw bytes as-is). */
template<typename T>
bool sendStruct(uint16_t id, uint8_t cmd, const T& pod) {
  return send(id, cmd, &pod, (uint8_t)sizeof(T));
}
template<typename T>
bool sendStruct(uint8_t cmd, const T& pod) {
  return sendDefault(cmd, &pod, (uint8_t)sizeof(T));
}

// ------------------ Little-endian READ helpers (for RX parsing) ------------------
inline bool readU8 (const uint8_t* b, size_t len, size_t off, uint8_t&  out) {
  if (off + 1 > len) return false; out = b[off]; return true;
}
inline bool readU16(const uint8_t* b, size_t len, size_t off, uint16_t& out) {
  if (off + 2 > len) return false; out = (uint16_t)b[off] | ((uint16_t)b[off+1] << 8); return true;
}
inline bool readU32(const uint8_t* b, size_t len, size_t off, uint32_t& out) {
  if (off + 4 > len) return false;
  out = (uint32_t)b[off]
      | ((uint32_t)b[off+1] << 8)
      | ((uint32_t)b[off+2] << 16)
      | ((uint32_t)b[off+3] << 24);
  return true;
}
inline bool readF32(const uint8_t* b, size_t len, size_t off, float& out) {
  if (off + 4 > len) return false; // STM32 is little-endian
  uint32_t v; if (!readU32(b, len, off, v)) return false;
  memcpy(&out, &v, 4); return true;
}

}
