#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t kernel_clock_hz;
    uint32_t nominal_bitrate;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t last_error_code;
    bool error_passive;
    bool warning;
    bool bus_off;
} CAN_InterfaceStatus;

bool CAN_Interface_Init(FDCAN_HandleTypeDef *hfdcan);
bool CAN_Send(uint16_t standard_id, const uint8_t *data, uint8_t length);
bool CAN_Receive(uint16_t *standard_id, uint8_t *data, uint8_t *length);
bool CAN_GetStatus(CAN_InterfaceStatus *status);
bool CAN_RunInternalLoopbackTest(void);

#endif
