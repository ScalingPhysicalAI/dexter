#include "can_interface.h"

static FDCAN_HandleTypeDef *s_can;

bool CAN_Interface_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};
    s_can = hfdcan;
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
    if (!s_can || HAL_FDCAN_GetRxFifoFillLevel(s_can, FDCAN_RX_FIFO0) == 0) return false;
    if (HAL_FDCAN_GetRxMessage(s_can, FDCAN_RX_FIFO0, &h, data) != HAL_OK) return false;
    *standard_id = (uint16_t)h.Identifier;
    *length = (uint8_t)(h.DataLength >> 16);
    return true;
}
