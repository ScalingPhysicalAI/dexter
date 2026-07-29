#include "main.h"
#include "limit_switches.h"

static volatile bool s_active_high = true;
static volatile bool s_enabled = false;

void Limit_SetEnabled(bool enabled)
{
    s_enabled = enabled;
}

bool Limit_GetEnabled(void)
{
    return s_enabled;
}

void Limit_SetActiveHigh(bool active_high)
{
    s_active_high = active_high;
}

bool Limit_GetActiveHigh(void)
{
    return s_active_high;
}

static bool limit_pin_active(GPIO_TypeDef *port, uint16_t pin)
{
    bool pin_high = HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
    return pin_high == s_active_high;
}

bool Limit_ZMinActive(void)
{
    return limit_pin_active(Z_LIMIT_MIN_GPIO_Port, Z_LIMIT_MIN_Pin);
}

bool Limit_ZMaxActive(void)
{
    return limit_pin_active(Z_LIMIT_MAX_GPIO_Port, Z_LIMIT_MAX_Pin);
}

bool Limit_ZMotionAllowed(bool positive_direction)
{
    if (!s_enabled) return true;
    return positive_direction ? !Limit_ZMaxActive() : !Limit_ZMinActive();
}
