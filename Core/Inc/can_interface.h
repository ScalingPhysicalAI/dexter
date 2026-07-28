#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool CAN_Interface_Init(FDCAN_HandleTypeDef *hfdcan);
bool CAN_Send(uint16_t standard_id, const uint8_t *data, uint8_t length);
bool CAN_Receive(uint16_t *standard_id, uint8_t *data, uint8_t *length);

#endif
