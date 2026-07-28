#ifndef COMMAND_IO_H
#define COMMAND_IO_H

#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMMAND_SOURCE_USB = 0,
    COMMAND_SOURCE_UART = 1
} CommandSource;

void CommandIO_Init(UART_HandleTypeDef *huart);
void CommandIO_Poll(void);
void CommandIO_Send(CommandSource source, const char *data, uint16_t length);

#endif
