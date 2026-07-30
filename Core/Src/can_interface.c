#include "can_interface.h"
#include <string.h>

static FDCAN_HandleTypeDef *s_can;

static bool configure_and_start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(s_can, &filter) != HAL_OK) return false;
    if (HAL_FDCAN_ConfigGlobalFilter(s_can, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return false;
    if (HAL_FDCAN_Start(s_can) != HAL_OK) return false;
    return HAL_FDCAN_ActivateNotification(s_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK;
}

bool CAN_Interface_Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) return false;
    s_can = hfdcan;
    return configure_and_start();
}

bool CAN_Send(uint16_t standard_id, const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef h = {0};
    static const uint32_t dlc[] = {FDCAN_DLC_BYTES_0,FDCAN_DLC_BYTES_1,FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,FDCAN_DLC_BYTES_4,FDCAN_DLC_BYTES_5,FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,FDCAN_DLC_BYTES_8};
    if (!s_can || length > 8) return false;
    h.Identifier = standard_id & 0x7ffU;
    h.IdType = FDCAN_STANDARD_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = dlc[length];
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = FDCAN_BRS_OFF;
    h.FDFormat = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(s_can, &h, (uint8_t *)data) == HAL_OK;
}

bool CAN_Receive(uint16_t *standard_id, uint8_t *data, uint8_t *length)
{
    FDCAN_RxHeaderTypeDef h;
    if (!s_can || !standard_id || !data || !length ||
        HAL_FDCAN_GetRxFifoFillLevel(s_can, FDCAN_RX_FIFO0) == 0U) return false;
    if (HAL_FDCAN_GetRxMessage(s_can, FDCAN_RX_FIFO0, &h, data) != HAL_OK) return false;
    *standard_id = (uint16_t)h.Identifier;
    *length = (uint8_t)(h.DataLength >> 16);
    return true;
}

bool CAN_GetStatus(CAN_InterfaceStatus *status)
{
    FDCAN_ProtocolStatusTypeDef protocol;
    FDCAN_ErrorCountersTypeDef counters;
    uint32_t divider;
    uint32_t time_quanta;

    if (s_can == NULL || status == NULL) return false;
    if (HAL_FDCAN_GetProtocolStatus(s_can, &protocol) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(s_can, &counters) != HAL_OK) return false;

    status->kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    divider = (s_can->Init.ClockDivider == FDCAN_CLOCK_DIV1) ?
              1U : (2U * s_can->Init.ClockDivider);
    time_quanta = 1U + s_can->Init.NominalTimeSeg1 + s_can->Init.NominalTimeSeg2;
    status->nominal_bitrate = status->kernel_clock_hz /
        (divider * s_can->Init.NominalPrescaler * time_quanta);
    status->tx_error_count = counters.TxErrorCnt;
    status->rx_error_count = counters.RxErrorCnt;
    status->last_error_code = protocol.LastErrorCode;
    status->error_passive = protocol.ErrorPassive != 0U;
    status->warning = protocol.Warning != 0U;
    status->bus_off = protocol.BusOff != 0U;
    return true;
}

bool CAN_RunInternalLoopbackTest(void)
{
    static const uint8_t expected[8] = {'D', 'E', 'X', 'T', 'E', 'R', 'L', '5'};
    uint8_t received[8] = {0};
    uint16_t identifier = 0U;
    uint8_t length = 0U;
    uint32_t original_mode;
    uint32_t started_at;
    bool test_passed = false;
    bool restored;

    if (s_can == NULL) return false;
    original_mode = s_can->Init.Mode;

    if (HAL_FDCAN_Stop(s_can) != HAL_OK || HAL_FDCAN_DeInit(s_can) != HAL_OK) {
        return false;
    }

    s_can->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    if (HAL_FDCAN_Init(s_can) == HAL_OK && configure_and_start() &&
        CAN_Send(0x5A5U, expected, sizeof(expected))) {
        started_at = HAL_GetTick();
        do {
            if (CAN_Receive(&identifier, received, &length)) {
                test_passed = identifier == 0x5A5U && length == sizeof(expected) &&
                              memcmp(received, expected, sizeof(expected)) == 0;
                break;
            }
        } while ((HAL_GetTick() - started_at) < 100U);
    }

    (void)HAL_FDCAN_Stop(s_can);
    (void)HAL_FDCAN_DeInit(s_can);
    s_can->Init.Mode = original_mode;
    restored = HAL_FDCAN_Init(s_can) == HAL_OK && configure_and_start();
    return test_passed && restored;
}
