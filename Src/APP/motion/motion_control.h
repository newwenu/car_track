#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "stm32f10x.h"

/* 运行方向 */
typedef enum {
    MOTION_DIR_FORWARD = 0, /* 前进 */
    MOTION_DIR_BACKWARD,    /* 后退 */
    MOTION_DIR_LEFT,        /* 原地左转：左轮后退，右轮前进 */
    MOTION_DIR_RIGHT,       /* 原地右转：左轮前进，右轮后退 */
    MOTION_DIR_MAX
} motion_dir_t;

/* 运动状态 */
typedef enum {
    MOTION_STATE_STOP = 0,  /* 停止 / 滑行 */
    MOTION_STATE_RUN,       /* 运行中 */
    MOTION_STATE_BRAKE,     /* 刹车（能耗制动） */
    MOTION_STATE_MAX
} motion_state_t;

/* 初始化：默认停车 */
void motion_init(void);

/* 周期性更新：建议每 APP_MOTOR_PERIOD_MS 调用一次，完成加减速斜坡 */
void motion_update(void);

/* 直接设置左右轮目标速度百分比：-100 ~ +100，正负代表方向 */
void motion_set_wheels(int left_pct, int right_pct);

/* 按指定方向和速度运行：speed_pct 范围 0 ~ 100 */
void motion_run_dir(motion_dir_t dir, int speed_pct);

/* 无级调速：直接设置整车目标速度百分比（前进为正，后退为负） */
void motion_set_speed(int speed_pct);

/* 滑行停止：速度斜坡降到 0 后进入 STOP 状态 */
void motion_stop(void);

/* 紧急刹车：立即短接电机两端制动 */
void motion_brake(void);

/* 获取当前运动状态 */
motion_state_t motion_get_state(void);

/* 获取当前左右轮实际速度百分比 */
void motion_get_current(int *left_pct, int *right_pct);

/* 获取当前左右轮目标速度百分比 */
void motion_get_target(int *left_pct, int *right_pct);

#endif
