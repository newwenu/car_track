#ifndef __TRACE_CONTROL_H
#define __TRACE_CONTROL_H

#include "stm32f10x.h"

/* 初始化循迹控制模块 */
void trace_control_init(void);

/* 周期性读取传感器并更新误差、目标轮速 */
void trace_control_update(void);

/* 获取当前循迹误差，范围约 -2 ~ +2 */
float trace_control_get_error(void);

/* 判断是否所有传感器都在黑线上（A 点/起点线特征） */
u8 trace_control_is_all_black(void);

/* 获取 PD 控制后的目标轮速百分比 */
void trace_control_get_wheel_targets(int *left_pct, int *right_pct);

/* 判断是否处于丢线状态（连续多次未检测到黑线） */
u8 trace_control_is_lost(void);

/* 判断丢线后寻线是否已超时 */
u8 trace_control_is_lost_timeout(void);

/* 获取上一次有效误差（用于观察 D 项与丢线惯性） */
float trace_control_get_last_error(void);

/* 获取丢线计数器：连续丢线次数与寻线恢复计数 */
void trace_control_get_lost_info(u8 *lost_cnt, u8 *recovery_cnt);

/* ===================== 阈值自学习接口 ===================== */

/* 启动阈值自学习：在约 2 秒内把传感器依次扫过黑线和白底 */
void trace_calib_start(void);

/* 返回自学习状态：0=未学习, 1=学习中, 2=成功, 3=失败 */
u8 trace_calib_get_state(void);

/* 学习是否成功：0=未成功，1=成功 */
u8 trace_calib_is_ok(void);

/* 获取指定通道学习到的最小/最大 ADC 值 */
void trace_calib_get_range(u8 idx, u16 *min_val, u16 *max_val);

/* 获取指定通道学习后的施密特阈值 */
void trace_calib_get_thresholds(u8 idx, u16 *th_high, u16 *th_low);

#endif
