#ifndef __APP_H
#define __APP_H

#include "stm32f10x.h"

/* 主循环节拍：10ms */
#define APP_TICK_MS             10

/* 各任务执行周期，必须是 APP_TICK_MS 的整数倍 */
#define APP_MOTOR_PERIOD_MS     10
#define APP_OBSTACLE_PERIOD_MS  10
#define APP_TRACE_PERIOD_MS     20
#define APP_FSM_PERIOD_MS       20
#define APP_UI_PERIOD_MS        50
#define APP_VEHICLE_PERIOD_MS   100

/* 兼容旧命名：APP_TASK_PERIOD_MS 原指 fsm 任务周期 */
#define APP_TASK_PERIOD_MS      APP_FSM_PERIOD_MS

void app_init(void);
void app_update(void);
u32  app_get_tick(void);

#endif
