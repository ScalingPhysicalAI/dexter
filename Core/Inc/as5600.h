#ifndef AS5600_H
#define AS5600_H

#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool online;
    bool valid;
    bool magnet_detected;
    bool magnet_too_weak;
    bool magnet_too_strong;
    bool inverted;
    uint16_t raw_angle;
    uint16_t angle_tenths_deg;
    int32_t multi_turn_counts;
    int32_t position_steps;
    uint32_t i2c_error_count;
    uint32_t last_update_ms;
} AS5600_Status;

void AS5600_Init(I2C_HandleTypeDef *hi2c);
void AS5600_Poll(void);
void AS5600_SetStepsPerRevolution(uint32_t steps_per_revolution);
void AS5600_SetInverted(bool inverted);
bool AS5600_GetInverted(void);
void AS5600_ZeroAtPosition(int32_t z_position_steps);
void AS5600_GetStatus(AS5600_Status *status);

#endif
