#ifndef __UI_DASHBOARD_H
#define __UI_DASHBOARD_H

#include "stm32f10x.h"

/* 界面页面枚举 */
typedef enum {
    UI_PAGE_MAIN = 0,   /* 主界面：速度 + 时间 + 里程 */
    UI_PAGE_MODE,       /* 模式设定界面 */
    UI_PAGE_DEBUG,      /* 调试界面：显示内部变量 / 测试功能 */
    UI_PAGE_MAX
} ui_page_t;

/* 运行模式枚举 */
typedef enum {
    UI_MODE_NORMAL = 0, /* 正常模式 */
    UI_MODE_RULE,       /* 规定模式 */
    UI_MODE_MAX
} ui_mode_t;

/* 初始化 OLED 并显示主界面 */
void ui_init(void);

/* 切换当前页面 */
void ui_switch_page(ui_page_t page);

/* 获取当前页面 */
ui_page_t ui_get_page(void);

/* 获取当前运行秒数 */
u16 ui_get_run_seconds(void);

/* 设置速度（范围 0~99） */
void ui_set_speed(u16 speed);

/* 设置里程（范围 0~99999） */
void ui_set_mileage(u32 mileage);

/* 设置/获取运行模式 */
void ui_set_mode(ui_mode_t mode);
ui_mode_t ui_get_mode(void);

/* 计时器控制（主界面时间显示 MM:SS） */
void ui_timer_start(void);
void ui_timer_stop(void);
void ui_timer_reset(void);

/* 轮询周期（单位：ms），main 中调用 ui_task 的时间间隔 */
#define UI_TASK_PERIOD_MS   50

/* 强制刷新当前页面动态数据 */
void ui_refresh(void);

/* 轮询按键并处理界面/模式切换；建议每 UI_TASK_PERIOD_MS 调用一次 */
void ui_task(void);

#endif
