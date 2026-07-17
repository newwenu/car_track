#ifndef __APP_H
#define __APP_H

#include "stm32f10x.h"

#define APP_TASK_PERIOD_MS  50

void app_init(void);
void app_update(void);

#endif
