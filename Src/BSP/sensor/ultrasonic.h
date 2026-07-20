#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"
#include "config.h"

void  ultrasonic_init(void);
void  ultrasonic_start(void);
float ultrasonic_get_distance(void);
float ultrasonic_measure_blocking(void);

/* 调试：获取原始回波时间、完成标志和监听状态 */
void ultrasonic_get_raw(u16 *echo_us, u8 *ok, u8 *listening);

/* 测距完成回调，应用层可重定义以快速响应（如紧急刹车）。
 * 参数 distance 为本次测距结果，单位 cm；0 表示无效。
 * [修复] 恢复此声明以兼容 obstacle_guard.c 的紧急刹车机制 */
void ultrasonic_distance_ready_callback(float distance);

#endif
