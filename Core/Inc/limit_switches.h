#ifndef LIMIT_SWITCHES_H
#define LIMIT_SWITCHES_H

#include <stdbool.h>

/* Inputs are normally closed: LOW = healthy, HIGH = open/tripped. */
bool Limit_ZMinActive(void);
bool Limit_ZMaxActive(void);
bool Limit_ZMotionAllowed(bool positive_direction);

#endif
