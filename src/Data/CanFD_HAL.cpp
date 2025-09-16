#include "CanFD_HAL.h"
#include <Main.h>
//========================== GPIO + Clock ==========================
// Change pins/AF to match your hardware if needed
static void FDCAN1_GPIO_Clock_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct{};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12; // PA11=RX, PA12=TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Returns the configured FDCAN kernel clock (Hz)
uint32_t FDCAN_SetKernelToPLLQ()
{
    __HAL_RCC_FDCAN_CLK_ENABLE(); // ensure FDCAN bus clock is on

    RCC_PeriphCLKInitTypeDef p{};
    p.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;

    // These macros vary by HAL version; one of these usually exists on G4:
#if defined(RCC_FDCANCLKSOURCE_PLLQ)
    p.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLLQ;
#elif defined(RCC_FDCANCLKSOURCE_PLL) // some HALs use this name for PLLQ
    p.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
#else
    // fallback: if no PLLQ route is available, you can try HSE (8 MHz) or PCLK1
    p.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE; // or RCC_FDCANCLKSOURCE_PCLK1
#endif

    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK)
    {
        return 0;
    }

#if defined(RCC_PERIPHCLK_FDCAN)
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
#else
    return 0;
#endif
}

//========================== Global handle =========================
extern "C"
{
    FDCAN_HandleTypeDef hfdcan1; // <-- this fixes the undefined reference
}
//========================== Timing solver =========================
// bitrate = f_ker / (Prescaler * (1 + TSEG1 + TSEG2))
// We choose total TQ and sample point; then find an integer prescaler.
// Works well for typical f_ker values (e.g., 48/80 MHz).
struct CanTiming
{
    uint16_t presc;
    uint8_t sjw;
    uint8_t tseg1;
    uint8_t tseg2;
};

static bool computeTiming(uint32_t fker, uint32_t bps,
                          uint8_t target_tq, float sample_point,
                          CanTiming &out)
{
    // sample_point is fraction in (0,1), e.g. 0.875 => 87.5%
    if (fker == 0 || bps == 0)
        return false;
    const uint32_t total_tq = target_tq; // 8..32 typical
    const float sp = (sample_point <= 0.0f) ? 0.8f : (sample_point >= 0.99f) ? 0.99f
                                                                             : sample_point;

    // place sample point within [tseg1, tseg2]
    // 1 (sync) + tseg1 + tseg2 = total_tq
    uint8_t tseg1 = (uint8_t)roundf((total_tq - 1) * sp);
    if (tseg1 < 1)
        tseg1 = 1;
    if (tseg1 > total_tq - 2)
        tseg1 = total_tq - 2;
    uint8_t tseg2 = (uint8_t)(total_tq - 1 - tseg1);
    if (tseg2 < 1)
        tseg2 = 1;

    // simple SJW = min(4, tseg2)
    uint8_t sjw = (tseg2 < 4) ? tseg2 : 4;

    // prescaler must be integer
    const uint32_t denom = (uint32_t)total_tq * bps;
    uint32_t presc = fker / denom;
    if (presc == 0)
        presc = 1;

    // refine presc to minimize error (try presc, presc+/-1)
    uint32_t bestPresc = presc;
    uint32_t bestErr = abs((int32_t)fker - (int32_t)(presc * denom));
    for (int d = -2; d <= 2; ++d)
    {
        if ((int32_t)presc + d <= 0)
            continue;
        uint32_t p = (uint32_t)((int32_t)presc + d);
        uint32_t err = abs((int32_t)fker - (int32_t)(p * denom));
        if (err < bestErr)
        {
            bestErr = err;
            bestPresc = p;
        }
    }
    presc = bestPresc;

    // bounds (per RM): Prescaler 1..512, TSEG1 1..256, TSEG2 1..128, SJW 1..128
    if (presc < 1 || presc > 512)
        return false;

    out.presc = (uint16_t)presc;
    out.sjw = sjw;
    out.tseg1 = tseg1;
    out.tseg2 = tseg2;
    return true;
}

// Get FDCAN kernel clock. On STM32G4 this is usually from RCC_PERIPHCLK_FDCAN.
// If HAL helper missing, fall back to a sane default like 80 MHz.
static uint32_t getFdcanKernelHz()
{
#if defined(RCC_PERIPHCLK_FDCAN)
    uint32_t hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    Debug.printf("FDCAN kernel clock: %lu Hz", hz);
    if (hz == 0)
        hz = 80000000UL;
    return hz;
#else
    return 80000000UL;
#endif
}

// Fill hfdcan1.Init timing from computed values
static void applyNominalTiming(FDCAN_HandleTypeDef &h, const CanTiming &t)
{
    Debug.printf("Nominal timing: presc=%u sjw=%u tseg1=%u tseg2=%u\n",
                 t.presc, t.sjw, t.tseg1, t.tseg2);
    h.Init.NominalPrescaler = t.presc;
    h.Init.NominalSyncJumpWidth = t.sjw;
    h.Init.NominalTimeSeg1 = t.tseg1;
    h.Init.NominalTimeSeg2 = t.tseg2;
}

static void applyDataTiming(FDCAN_HandleTypeDef &h, const CanTiming &t)
{
    Debug.printf("Data timing: presc=%u sjw=%u tseg1=%u tseg2=%u\n",
                 t.presc, t.sjw, t.tseg1, t.tseg2);
    h.Init.DataPrescaler = t.presc;
    h.Init.DataSyncJumpWidth = t.sjw;
    h.Init.DataTimeSeg1 = t.tseg1;
    h.Init.DataTimeSeg2 = t.tseg2;
}

//========================== HAL facade ==========================
namespace CanHAL
{

    bool begin(uint32_t nominal_bps, uint32_t data_bps, const CanIds &canCfg)
    {
        FDCAN1_GPIO_Clock_Init();
        FDCAN_SetKernelToPLLQ();

        const uint32_t fker = getFdcanKernelHz();

        // Basic configuration
        hfdcan1.Instance = FDCAN1;
        hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS; // FD + BRS
        hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
        hfdcan1.Init.AutoRetransmission = ENABLE;
        hfdcan1.Init.TransmitPause = DISABLE;
        hfdcan1.Init.ProtocolException = ENABLE;

        hfdcan1.Init.StdFiltersNbr = 1;
        hfdcan1.Init.ExtFiltersNbr = 1;
        hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

        // Compute timings
        CanTiming nom{}, dat{};
        // Good defaults: 16 TQ @ 87.5% sample for nominal, 8 TQ @ 75% for data
        if (!computeTiming(fker, nominal_bps, 17, 0.875f, nom))
            return false;
        if (!computeTiming(fker, data_bps, 17, 0.75f, dat))
            return false;

        applyNominalTiming(hfdcan1, nom);
        applyDataTiming(hfdcan1, dat);

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
            return false;

        // Global filter: reject everything not matching a filter
        if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                         FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT) != HAL_OK)
        {
            return false;
        }

        if (!setFilterExact(canCfg))
            return false;

        return (HAL_FDCAN_Start(&hfdcan1) == HAL_OK);
    }

    bool setFilterExact(const CanIds &canCfg)
    {
        FDCAN_FilterTypeDef f{};
        if (canCfg.isExtended)
        {
            f.IdType = FDCAN_EXTENDED_ID;
            f.FilterIndex = 0;
            f.FilterType = FDCAN_FILTER_MASK;
            f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            f.FilterID1 = canCfg.arbId & 0x1FFFFFFF;
            f.FilterID2 = 0x1FFFFFFF;
        }
        else
        {
            f.IdType = FDCAN_STANDARD_ID;
            f.FilterIndex = 0;
            f.FilterType = FDCAN_FILTER_MASK;
            f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            f.FilterID1 = canCfg.arbId & 0x7FF;
            f.FilterID2 = 0x7FF;
        }
        return HAL_FDCAN_ConfigFilter(&hfdcan1, &f) == HAL_OK;
    }

    bool read(CanPacket &out)
    {
        FDCAN_RxHeaderTypeDef rxh{};
        uint8_t data[64];

        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, data) != HAL_OK)
            return false; // empty

        if (rxh.FDFormat != FDCAN_FD_CAN)
            return false;
        if (rxh.DataLength != FDCAN_DLC_BYTES_32)
            return false;

        out.arb_id = (rxh.IdType == FDCAN_EXTENDED_ID)
                         ? (rxh.Identifier & 0x1FFFFFFF)
                         : (rxh.Identifier & 0x7FF);

        out.id16 = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
        out.cmd = data[2];
        out.len = 29;
        for (uint8_t i = 0; i < 29; ++i)
            out.payload[i] = data[3 + i];

        return true;
    }

    bool write(uint16_t id16, uint8_t cmd, const uint8_t *payload, uint8_t len,
               const CanIds &canCfg)
    {
        if (len > 29)
            len = 29;

        uint8_t data[32] = {0};
        data[0] = uint8_t(id16 & 0xFF);
        data[1] = uint8_t(id16 >> 8);
        data[2] = cmd;
        for (uint8_t i = 0; i < len; ++i)
            data[3 + i] = payload[i];

        FDCAN_TxHeaderTypeDef txh{};
        txh.Identifier = canCfg.arbId & (canCfg.isExtended ? 0x1FFFFFFF : 0x7FF);
        txh.IdType = canCfg.isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        txh.TxFrameType = FDCAN_DATA_FRAME;
        txh.FDFormat = FDCAN_FD_CAN;
        txh.BitRateSwitch = FDCAN_BRS_ON;
        txh.DataLength = FDCAN_DLC_BYTES_32;
        txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txh.MessageMarker = 0;

        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data) == HAL_OK;
    }
}