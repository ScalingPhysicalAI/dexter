/* usbd_cdc_if.c — USB CDC with TX ring buffer + RX ring buffer
 *
 * TX architecture (fixes DMA stall after a few commands):
 *   GCode_Send()      → copies bytes into tx_ring[] (never touches USB directly)
 *   USB_CDC_TxPoll()  → called from main loop; if TxState==0 and ring has data,
 *                        copies one chunk into UserTxBufferFS and fires DMA
 *   USBD_CDC_DataIn   → (middleware callback) calls USB_CDC_TxPoll via HAL hook
 *
 * This eliminates:
 *   - Single static buffer overwritten while DMA is reading it
 *   - HAL_Delay spin blocking the main loop
 *   - ZLP deadlock: we never send multiples of 64 bytes (pad to 63 if needed)
 *
 * RX architecture (unchanged):
 *   CDC_Receive_FS()  → ISR: bytes into rx_ring[]
 *   USB_CDC_DrainRX() → main loop: bytes out to GCode_PutChar()
 */

#include "usbd_cdc_if.h"
#include "gcode.h"
#include <string.h>

/* ── static USB DMA buffers ─────────────────────────────────────────────── */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];   /* DMA source — must be static  */

/* ── TX ring buffer ─────────────────────────────────────────────────────── */
#define TX_RING_SIZE  512                   /* power of 2                   */
#define TX_CHUNK_MAX   63                   /* < 64 (USB FS packet) avoids ZLP */

static volatile uint8_t  tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head = 0;      /* written by GCode_Send (main) */
static volatile uint16_t tx_tail = 0;      /* read    by USB_CDC_TxPoll    */

/* ── RX ring buffer ─────────────────────────────────────────────────────── */
#define RX_RING_SIZE  256                   /* power of 2                   */

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ── forward declarations ───────────────────────────────────────────────── */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* ══════════════════════════════════════════════════════════════════════════
 *  TX ring buffer — push bytes in (called from main loop / GCode_Send)
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * USB_CDC_TxWrite — push one string into TX ring buffer.
 * Safe to call from main loop context only.
 * Bytes dropped if ring is full (caller can retry or discard).
 */
void USB_CDC_TxWrite(const char *str, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (tx_head + 1) & (TX_RING_SIZE - 1);
        if (next == tx_tail) break;         /* ring full — drop remainder   */
        tx_ring[tx_head] = (uint8_t)str[i];
        tx_head = next;
    }
}

/**
 * USB_CDC_TxPoll — drain TX ring into USB DMA.
 * Call from main loop (GCode_Poll) and from DataIn completion.
 * Never blocks.
 */
void USB_CDC_TxPoll(void)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) return;

    /* Already transferring — DataIn callback will re-trigger us */
    if (hcdc->TxState != 0) return;

    /* Nothing to send */
    if (tx_tail == tx_head) return;

    /* Copy up to TX_CHUNK_MAX bytes from ring into static DMA buffer.
     * Keeping chunk < 64 bytes means the USB host sees a short packet
     * and knows the transfer is complete — no ZLP needed, no stall. */
    uint16_t count = 0;
    uint16_t tail  = tx_tail;

    while (tail != tx_head && count < TX_CHUNK_MAX) {
        UserTxBufferFS[count++] = tx_ring[tail];
        tail = (tail + 1) & (TX_RING_SIZE - 1);
    }

    tx_tail = tail;                         /* advance consumer pointer     */

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, count);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  RX ring buffer drain (called from GCode_Poll in main loop)
 * ══════════════════════════════════════════════════════════════════════════ */
void USB_CDC_DrainRX(void)
{
    while (rx_tail != rx_head) {
        uint8_t byte = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
        GCode_PutChar((char)byte);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CDC middleware callbacks
 * ══════════════════════════════════════════════════════════════════════════ */

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    (void)pbuf; (void)length; (void)cmd;
    return USBD_OK;
}

/**
 * CDC_Receive_FS — USB ISR context.
 * Push received bytes into RX ring ONLY. No parsing here.
 */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    uint32_t len = *Len;
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1) & (RX_RING_SIZE - 1);
        if (next != rx_tail) {
            rx_ring[rx_head] = Buf[i];
            rx_head = next;
        }
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

/**
 * CDC_Transmit_FS — kept for compatibility but routes through ring buffer.
 * Deprecated: use USB_CDC_TxWrite() + USB_CDC_TxPoll() instead.
 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
    USB_CDC_TxWrite((const char *)Buf, Len);
    USB_CDC_TxPoll();
    return USBD_OK;
}
