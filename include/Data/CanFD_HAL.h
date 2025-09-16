#pragma once
#include <Arduino.h>

// Select the right HAL family header for your MCU
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_fdcan.h>

// This is DEFINED in CanFD_HAL_port.cpp
extern "C" FDCAN_HandleTypeDef hfdcan1;

struct CanPacket
{
    uint32_t arb_id; // bus arbitration ID (11/29-bit)
    uint16_t id16;   // data[0..1] little-endian
    uint8_t cmd;     // data[2]
    uint8_t len;     // payload length (0..29)
    uint8_t payload[29];
};

namespace CanHAL
{

    struct CanIds
    {
        uint32_t arbId = 0x123;     // my arbitration ID
        bool isExtended = false;    // false=11-bit std, true=29-bit ext
        uint16_t nodeId16 = 0x0042; // in-frame node id (data[0..1])
    };

    // Initializes clocks + pins + timings and starts FDCAN1.
    // - nominal_bps: e.g. 500000 or 1000000
    // - data_bps:    e.g. 2000000
    // - canCfg:      your IDs for filter + send
    bool begin(uint32_t nominal_bps, uint32_t data_bps, const CanIds &canCfg);

    // Install exact-match hardware filter to accept only canCfg.arbId into RX FIFO0
    bool setFilterExact(const CanIds &canCfg);

    // Non-blocking read of a 32-byte FD frame from FIFO0.
    bool read(CanPacket &out);

    // Non-blocking write of a 32-byte FD frame (FD+BRS) using canCfg.arbId.
    bool write(uint16_t id16, uint8_t cmd, const uint8_t *payload, uint8_t len,
               const CanIds &canCfg);

} // namespace CanHAL