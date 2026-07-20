#ifndef __ALARM_CONTROL_H
#define __ALARM_CONTROL_H

#include "stm32f10x.h"

/* 报警事件类型 */
typedef enum {
    ALARM_EVENT_IDLE = 0,       /* 空闲/无报警 */
    ALARM_EVENT_START,          /* 启动提示 */
    ALARM_EVENT_A_POINT,        /* 过 A 点提示 */
    ALARM_EVENT_AVOIDING,       /* 避障中持续报警 */
    ALARM_EVENT_BRAKING,        /* A 点停车提示 */
    ALARM_EVENT_FINISHED        /* 完成提示 */
} alarm_event_t;

/* 初始化报警模块 */
void alarm_control_init(void);

/* 触发一次报警事件 */
void alarm_control_event(alarm_event_t event);

/* 周期性更新报警状态（蜂鸣器/LED 周期控制） */
void alarm_control_update(void);

/* LED_STAT 闪烁模式 */
typedef enum {
    LED_MODE_OFF = 0,       /* 常灭 */
    LED_MODE_STEADY_ON,     /* 常亮 */
    LED_MODE_SLOW_BLINK,    /* 慢闪 0.5Hz */
    LED_MODE_FAST_BLINK,    /* 快闪 4Hz */
    LED_MODE_DOUBLE_FLASH,  /* 双闪 */
    LED_MODE_BREATH,        /* 呼吸灯（软件模拟） */
    LED_MODE_MAX
} led_mode_t;

/* 设置 LED_STAT 主显示模式，通常由 FSM 状态调用 */
void alarm_control_set_stat_mode(led_mode_t mode);

#endif
