#include "main.h"
#include "limit_switches.h"

bool Limit_ZMinActive(void)
{
    return HAL_GPIO_ReadPin(Z_LIMIT_MIN_GPIO_Port, Z_LIMIT_MIN_Pin) == GPIO_PIN_SET;
}

bool Limit_ZMaxActive(void)
{
    return HAL_GPIO_ReadPin(Z_LIMIT_MAX_GPIO_Port, Z_LIMIT_MAX_Pin) == GPIO_PIN_SET;
}

bool Limit_ZMotionAllowed(bool positive_direction)
{
    return positive_direction ? !Limit_ZMaxActive() : !Limit_ZMinActive();
}
