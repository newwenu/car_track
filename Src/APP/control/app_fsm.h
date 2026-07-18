#ifndef __APP_FSM_H
#define __APP_FSM_H

#include "stm32f10x.h"

/* 整车运行状态 */
typedef enum {
    FSM_STATE_IDLE = 0,     /* 待机，等待启动 */
    FSM_STATE_RUNNING,      /* 正常循迹行驶 */
    FSM_STATE_AVOIDING,     /* 前方障碍，紧急刹车+声光报警 */
    FSM_STATE_BRAKING,      /* A点临时停车 / 规则模式停顿 */
    FSM_STATE_FINISHED      /* 完成2圈，停车 */
} fsm_state_t;

/* 初始化状态机，默认进入 IDLE */
void fsm_init(void);

/* 主循环调用，建议每 APP_FSM_PERIOD_MS 调用一次 */
void fsm_update(void);

/* 获取当前状态 */
fsm_state_t fsm_get_state(void);

/* 触发启动（可由按键或声控调用） */
void fsm_start(void);

/* 重置为 IDLE，清空圈数、计时等 */
void fsm_reset(void);

/* 紧急刹车入口：由 app.c 的独立刹车块调用，同步切到 AVOIDING 状态 */
void fsm_handle_emergency_brake(void);

#endif
