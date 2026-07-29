#ifndef COMMAND_IO_H
#define COMMAND_IO_H

#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMMAND_SOURCE_USB = 0,
    COMMAND_SOURCE_UART = 1,
    COMMAND_SOURCE_CAN = 2,
    COMMAND_SOURCE_COUNT
} CommandSource;

/* Classic CAN ASCII command stream. Commands on RX ID must end with '\n'. */
#ifndef CAN_COMMAND_RX_ID
#define CAN_COMMAND_RX_ID 0x600U
#endif

#ifndef CAN_COMMAND_TX_ID
#define CAN_COMMAND_TX_ID 0x601U
#endif

void CommandIO_Init(UART_HandleTypeDef *huart);
void CommandIO_Poll(void);
void CommandIO_Send(CommandSource source, const char *data, uint16_t length);
bool CommandIO_SetCanIds(uint16_t receive_id, uint16_t transmit_id);
void CommandIO_GetCanIds(uint16_t *receive_id, uint16_t *transmit_id);

#endif
