#include "command_io.h"
#include "can_interface.h"
#include "gcode.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define UART_RX_RING_SIZE 256U
#define UART_TX_RING_SIZE 1024U
#define UART_TX_CHUNK_SIZE 64U
#define CAN_TX_RING_SIZE 1024U
#define CAN_FRAME_MAX_BYTES 8U

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
static uint8_t s_can_tx_ring[CAN_TX_RING_SIZE];
static uint16_t s_can_tx_head;
static uint16_t s_can_tx_tail;
static uint16_t s_can_receive_id;
static uint16_t s_can_transmit_id;
static uint16_t s_can_pending_receive_id;
static uint16_t s_can_pending_transmit_id;
static bool s_can_id_change_pending;

void CommandIO_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_rx_head = s_rx_tail = 0U;
    s_tx_head = s_tx_tail = 0U;
    s_can_tx_head = s_can_tx_tail = 0U;
    s_can_receive_id = CAN_COMMAND_RX_ID;
    s_can_transmit_id = CAN_COMMAND_TX_ID;
    s_can_pending_receive_id = CAN_COMMAND_RX_ID;
    s_can_pending_transmit_id = CAN_COMMAND_TX_ID;
    s_can_id_change_pending = false;
    s_tx_busy = false;
    if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

bool CommandIO_SetCanIds(uint16_t receive_id, uint16_t transmit_id)
{
    if (receive_id > 0x7FFU || transmit_id > 0x7FFU || receive_id == transmit_id) {
        return false;
    }
    s_can_pending_receive_id = receive_id;
    s_can_pending_transmit_id = transmit_id;
    s_can_id_change_pending = true;
    return true;
}

void CommandIO_GetCanIds(uint16_t *receive_id, uint16_t *transmit_id)
{
    if (receive_id != NULL) {
        *receive_id = s_can_id_change_pending ? s_can_pending_receive_id : s_can_receive_id;
    }
    if (transmit_id != NULL) {
        *transmit_id = s_can_id_change_pending ? s_can_pending_transmit_id : s_can_transmit_id;
    }
}

void CommandIO_Send(CommandSource source, const char *data, uint16_t length)
{
    if (source == COMMAND_SOURCE_USB) {
        USB_CDC_TxWrite(data, length);
        return;
    }
    if (source == COMMAND_SOURCE_CAN) {
        for (uint16_t i = 0U; i < length; ++i) {
            uint16_t next = (uint16_t)((s_can_tx_head + 1U) & (CAN_TX_RING_SIZE - 1U));
            if (next == s_can_tx_tail) break;
            s_can_tx_ring[s_can_tx_head] = (uint8_t)data[i];
            s_can_tx_head = next;
        }
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

static void can_rx_poll(void)
{
    uint16_t standard_id;
    uint8_t data[CAN_FRAME_MAX_BYTES];
    uint8_t length;
    while (CAN_Receive(&standard_id, data, &length)) {
        if (standard_id != s_can_receive_id) continue;
        for (uint8_t index = 0U; index < length; ++index) {
            GCode_PutCharFrom(COMMAND_SOURCE_CAN, (char)data[index]);
        }
    }
}

static void can_tx_poll(void)
{
    if (s_can_tx_tail == s_can_tx_head) {
        if (s_can_id_change_pending) {
            s_can_receive_id = s_can_pending_receive_id;
            s_can_transmit_id = s_can_pending_transmit_id;
            s_can_id_change_pending = false;
        }
        return;
    }

    uint8_t data[CAN_FRAME_MAX_BYTES];
    uint8_t length = 0U;
    uint16_t cursor = s_can_tx_tail;
    bool flush_now = false;

    while (cursor != s_can_tx_head && length < CAN_FRAME_MAX_BYTES) {
        char current_char = (char)s_can_tx_ring[cursor];
        data[length++] = (uint8_t)current_char;

        if (current_char == '\n' || current_char == '\r') {
            flush_now = true;
        }

        cursor = (uint16_t)((cursor + 1U) & (CAN_TX_RING_SIZE - 1U));
    }

    if (length == CAN_FRAME_MAX_BYTES || flush_now || cursor == s_can_tx_head) {
        if (CAN_Send(s_can_transmit_id, data, length)) {
            s_can_tx_tail = cursor;
        }
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
    can_rx_poll();
    uart_tx_poll();
    can_tx_poll();
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
