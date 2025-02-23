#ifndef CLOCK_CONFIG_H
#define CLOCK_CONFIG_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif // CLOCK_CONFIG_H