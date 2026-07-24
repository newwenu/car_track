#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "config.h"

/* ========================================================================
 * 编码器模块说明：
 *
 * 硬件配置：同轴双轮驱动（无差速比）
 * - 左右轮直接同轴连接电机
 * - 差速比 = 1:1
 *
 * 信号处理流程：
 * 1. GPIO 中断捕获编码器脉冲（带消抖）
 * 2. 低通滤波（一阶 IIR）抑制高频噪声
 * 3. 输出平滑后的脉冲计数值
 *
 * 接口函数：
 * - encoder_read(): 获取滤波后的脉冲增量（左右轮之和），并清零
 * ======================================================================== */

void encoder_init(void);
void encoder_left_inc(void);
void encoder_right_inc(void);
int  encoder_read(void);

/* 调试：获取上电以来左右轮累计脉冲总数（不清零，未经过滤波）。
 * 注意：车速/里程模块使用 encoder_read() 取走并清零增量计数，
 *       但本函数返回的是独立维护的累计总量，不受 encoder_read() 影响。 */
void encoder_get_counts(s32 *left, s32 *right);

/* 调试：清零累计总计数，用于 PPR 测量等场景 */
void encoder_reset_totals(void);

#endif
