#include "command_io.h"
#include "gcode.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define UART_RX_RING_SIZE 256U
#define UART_TX_RING_SIZE 1024U
#define UART_TX_CHUNK_SIZE 64U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_tx_ring[UART_TX_RING_SIZE];
static uint16_t s_tx_head;
static uint16_t s_tx_tail;
static uint8_t s_tx_chunk[UART_TX_CHUNK_SIZE];
static volatile bool s_tx_busy;

void CommandIO_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_rx_head = s_rx_tail = 0U;
    s_tx_head = s_tx_tail = 0U;
    s_tx_busy = false;
    if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void CommandIO_Send(CommandSource source, const char *data, uint16_t length)
{
    if (source == COMMAND_SOURCE_USB) {
        USB_CDC_TxWrite(data, length);
        return;
    }

    for (uint16_t i = 0U; i < length; ++i) {
        uint16_t next = (uint16_t)((s_tx_head + 1U) & (UART_TX_RING_SIZE - 1U));
        if (next == s_tx_tail) break;
        s_tx_ring[s_tx_head] = (uint8_t)data[i];
        s_tx_head = next;
    }
}

static void uart_tx_poll(void)
{
    if (s_uart == NULL || s_tx_busy || s_tx_tail == s_tx_head) return;

    uint16_t count = 0U;
    while (s_tx_tail != s_tx_head && count < UART_TX_CHUNK_SIZE) {
        s_tx_chunk[count++] = s_tx_ring[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) & (UART_TX_RING_SIZE - 1U));
    }
    s_tx_busy = true;
    if (HAL_UART_Transmit_IT(s_uart, s_tx_chunk, count) != HAL_OK) {
        s_tx_busy = false;
    }
}

void CommandIO_Poll(void)
{
    USB_CDC_RxPoll();
    while (s_rx_tail != s_rx_head) {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (UART_RX_RING_SIZE - 1U));
        GCode_PutCharFrom(COMMAND_SOURCE_UART, (char)byte);
    }
    uart_tx_poll();
    USB_CDC_TxPoll();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_uart) return;
    uint16_t next = (uint16_t)((s_rx_head + 1U) & (UART_RX_RING_SIZE - 1U));
    if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_uart) s_tx_busy = false;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_uart) {
        s_tx_busy = false;
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}
