#ifndef __UI_DEBUG_H
#define __UI_DEBUG_H

#include "stm32f10x.h"

/* DEBUG 页面下的测试/显示项 */
typedef enum {
    UI_DEBUG_INFO = 0,  /* 综合内部变量 */
    UI_DEBUG_TRACE,     /* 循迹原始数据 */
    /* UI_DEBUG_STATE,  系统状态页已暂时移除，简化调试界面 */
    UI_DEBUG_ACT,       /* 外部报警 LED + 蜂鸣器自检 */
    UI_DEBUG_MOTOR,     /* 电机 IO 自检（原地左转/右转） */
    UI_DEBUG_ULTRASONIC,/* 超声原始数据与上下拉测试 */
    UI_DEBUG_MAX
} ui_debug_page_t;

/* 进入/退出 DEBUG 页面 */
void ui_debug_enter(void);
void ui_debug_exit(void);

/* 短按切换下一个测试项 */
void ui_debug_next(void);

/* 刷新显示，建议每 UI_TASK_PERIOD_MS 调用一次 */
void ui_debug_update(void);

/* 立即重绘当前 DEBUG 页面 */
void ui_debug_draw(void);

/* 获取当前 DEBUG 子页面 */
ui_debug_page_t ui_debug_get_page(void);

/* 电机测试是否处于激活状态（供 FSM 在 IDLE 时跳过 motion_stop） */
u8 ui_debug_motor_active(void);

/* 当前是否处于 DEBUG 页面（开关标志，供外层决定是否调用 ui_debug_update） */
u8 ui_debug_is_active(void);

/* 超声调试页：切换回波引脚上下拉；非超声页调用无效 */
void ui_debug_toggle_ultrasonic_pull(void);

#endif
