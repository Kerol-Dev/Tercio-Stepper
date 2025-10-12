#pragma once

#include <Arduino.h>

#ifndef DEVELOPER_MODE
#define DEVELOPER_MODE 1
#endif

constexpr bool kDeveloperMode = (DEVELOPER_MODE != 0);

// ---------- Debug sink ----------
#if DEVELOPER_MODE
// STM32duino HardwareSerial with explicit RX/TX pins: RX=PA3, TX=PA2
inline HardwareSerial Debug(PA3, PA2);
#else
// Minimal no-op stream so code compiles even without debug
class NullStream : public Print {
public:
  void begin(unsigned long) {}
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t*, size_t n) override { return n; }
  using Print::write;
  template <typename... Args> void printf(const char*, Args...) {}
};
inline NullStream Debug;
#endif

// ---------- Debug macros ----------
#if DEVELOPER_MODE
  #define DBG_INIT(baud)   do { Debug.begin(baud); } while (0)
  #define DBG_PRINTLN(msg) do { Debug.println(msg); } while (0)
  #define DBG_PRINTF(...)  do { Debug.printf(__VA_ARGS__); } while (0)
#else
  #define DBG_INIT(baud)   do { } while (0)
  #define DBG_PRINTLN(msg) do { } while (0)
  #define DBG_PRINTF(...)  do { } while (0)
#endif