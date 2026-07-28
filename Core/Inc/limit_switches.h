#ifndef LIMIT_SWITCHES_H
#define LIMIT_SWITCHES_H

#include <stdbool.h>

/* Default: active HIGH (normally-closed switch, HIGH=open/tripped). */
void Limit_SetActiveHigh(bool active_high);
bool Limit_GetActiveHigh(void);
bool Limit_ZMinActive(void);
bool Limit_ZMaxActive(void);
bool Limit_ZMotionAllowed(bool positive_direction);

#endif
