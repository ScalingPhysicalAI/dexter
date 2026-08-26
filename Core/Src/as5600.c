#include "as5600.h"
#include <string.h>

#define AS5600_I2C_ADDRESS        (0x36U << 1U)
#define AS5600_STATUS_REGISTER    0x0BU
#define AS5600_STATUS_MD          (1U << 5U)
#define AS5600_STATUS_ML          (1U << 4U)
#define AS5600_STATUS_MH          (1U << 3U)
#define AS5600_COUNTS_PER_TURN    4096L
#define AS5600_HALF_TURN_COUNTS   2048L
#define AS5600_POLL_INTERVAL_MS   10U
#define AS5600_OFFLINE_RETRY_MS   100U
#define AS5600_I2C_TIMEOUT_MS     5U

static I2C_HandleTypeDef *s_i2c;
static AS5600_Status s_status;
static uint32_t s_steps_per_revolution = 3200U;
static uint32_t s_last_attempt_ms;
static uint16_t s_previous_raw;
static int32_t s_position_origin_steps;
static bool s_have_sample;

void AS5600_Init(I2C_HandleTypeDef *hi2c)
{
    s_i2c = hi2c;
    memset(&s_status, 0, sizeof(s_status));
    s_last_attempt_ms = HAL_GetTick() - AS5600_OFFLINE_RETRY_MS;
    s_previous_raw = 0U;
    s_position_origin_steps = 0;
    s_have_sample = false;
}

void AS5600_SetStepsPerRevolution(uint32_t steps_per_revolution)
{
    if (steps_per_revolution != 0U) s_steps_per_revolution = steps_per_revolution;
}

void AS5600_SetInverted(bool inverted)
{
    s_status.inverted = inverted;
}

bool AS5600_GetInverted(void)
{
    return s_status.inverted;
}

void AS5600_ZeroAtPosition(int32_t z_position_steps)
{
    s_position_origin_steps = z_position_steps;
    s_status.multi_turn_counts = 0;
    s_status.position_steps = z_position_steps;
    if (s_status.online && s_status.valid) {
        s_previous_raw = s_status.raw_angle;
        s_have_sample = true;
    } else {
        s_have_sample = false;
    }
}

void AS5600_GetStatus(AS5600_Status *status)
{
    if (status != NULL) *status = s_status;
}

void AS5600_Poll(void)
{
    if (s_i2c == NULL) return;

    uint32_t now = HAL_GetTick();
    uint32_t interval = s_status.online ? AS5600_POLL_INTERVAL_MS :
                                         AS5600_OFFLINE_RETRY_MS;
    if ((uint32_t)(now - s_last_attempt_ms) < interval) return;
    s_last_attempt_ms = now;

    uint8_t registers[3];
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(s_i2c, AS5600_I2C_ADDRESS,
                                                AS5600_STATUS_REGISTER,
                                                I2C_MEMADD_SIZE_8BIT,
                                                registers, sizeof(registers),
                                                AS5600_I2C_TIMEOUT_MS);
    if (result != HAL_OK) {
        s_status.online = false;
        s_status.valid = false;
        ++s_status.i2c_error_count;
        return;
    }

    uint8_t sensor_status = registers[0];
    uint16_t raw = (uint16_t)((((uint16_t)registers[1] & 0x0FU) << 8U) |
                              (uint16_t)registers[2]);
    s_status.online = true;
    s_status.magnet_detected = (sensor_status & AS5600_STATUS_MD) != 0U;
    s_status.magnet_too_weak = (sensor_status & AS5600_STATUS_ML) != 0U;
    s_status.magnet_too_strong = (sensor_status & AS5600_STATUS_MH) != 0U;
    s_status.valid = s_status.magnet_detected && !s_status.magnet_too_weak &&
                     !s_status.magnet_too_strong;
    s_status.raw_angle = raw;
    s_status.angle_tenths_deg = (uint16_t)(((uint32_t)raw * 3600U) /
                                           (uint32_t)AS5600_COUNTS_PER_TURN);
    s_status.last_update_ms = now;

    if (!s_status.valid) return;
    if (!s_have_sample) {
        s_previous_raw = raw;
        s_have_sample = true;
        s_status.position_steps = s_position_origin_steps;
        return;
    }

    int32_t delta = (int32_t)raw - (int32_t)s_previous_raw;
    if (delta > AS5600_HALF_TURN_COUNTS) delta -= AS5600_COUNTS_PER_TURN;
    else if (delta < -AS5600_HALF_TURN_COUNTS) delta += AS5600_COUNTS_PER_TURN;
    s_previous_raw = raw;
    if (s_status.inverted) delta = -delta;
    s_status.multi_turn_counts += delta;
    s_status.position_steps = s_position_origin_steps +
        (int32_t)(((int64_t)s_status.multi_turn_counts * s_steps_per_revolution) /
                  AS5600_COUNTS_PER_TURN);
}
