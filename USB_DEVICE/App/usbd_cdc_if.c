#include "usbd_cdc_if.h"
#include "command_io.h"
#include "gcode.h"

#define USB_TX_RING_SIZE 1024U
#define USB_RX_RING_SIZE 512U
#define USB_TX_CHUNK_SIZE 63U

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static uint8_t s_tx_ring[USB_TX_RING_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static uint8_t s_rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length);
static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length, uint8_t endpoint);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
    /* Keep data queued before USB enumeration, including the boot command guide. */
    s_rx_head = s_rx_tail = 0U;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) { return USBD_OK; }

static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length)
{
    (void)command;
    (void)buffer;
    (void)length;
    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
    for (uint32_t index = 0U; index < *length; ++index) {
        uint16_t next = (uint16_t)((s_rx_head + 1U) & (USB_RX_RING_SIZE - 1U));
        if (next == s_rx_tail) break;
        s_rx_ring[s_rx_head] = buffer[index];
        s_rx_head = next;
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length, uint8_t endpoint)
{
    (void)buffer;
    (void)length;
    (void)endpoint;
    return USBD_OK;
}

void USB_CDC_TxWrite(const char *data, uint16_t length)
{
    for (uint16_t index = 0U; index < length; ++index) {
        uint16_t next = (uint16_t)((s_tx_head + 1U) & (USB_TX_RING_SIZE - 1U));
        if (next == s_tx_tail) break;
        s_tx_ring[s_tx_head] = (uint8_t)data[index];
        s_tx_head = next;
    }
}

void USB_CDC_TxPoll(void)
{
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (cdc == NULL || cdc->TxState != 0U || s_tx_tail == s_tx_head) return;

    uint16_t count = 0U;
    while (s_tx_tail != s_tx_head && count < USB_TX_CHUNK_SIZE) {
        UserTxBufferFS[count++] = s_tx_ring[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) & (USB_TX_RING_SIZE - 1U));
    }
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, count);
    (void)USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

void USB_CDC_RxPoll(void)
{
    while (s_rx_tail != s_rx_head) {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (USB_RX_RING_SIZE - 1U));
        GCode_PutCharFrom(COMMAND_SOURCE_USB, (char)byte);
    }
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    USB_CDC_TxWrite((const char *)buffer, length);
    USB_CDC_TxPoll();
    return USBD_OK;
}
