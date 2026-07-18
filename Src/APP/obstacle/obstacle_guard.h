#ifndef __OBSTACLE_GUARD_H
#define __OBSTACLE_GUARD_H

#include "stm32f10x.h"

/* 初始化避障检测模块 */
void obstacle_guard_init(void);

/* 周期性读取超声波距离并更新状态 */
void obstacle_guard_update(void);

/* 当前是否触发避障（连续检测到障碍） */
u8 obstacle_guard_is_triggered(void);

/* 当前是否已解除障碍（连续无障碍） */
u8 obstacle_guard_is_cleared(void);

/* 获取最近一次测距结果（cm），0 表示无有效值 */
float obstacle_guard_get_distance(void);

/* 紧急刹车请求：由超声波中断设置，app 主循环查询后清除 */
void obstacle_guard_set_emergency_request(void);
u8   obstacle_guard_has_emergency_request(void);
void obstacle_guard_clear_emergency_request(void);

#endif
