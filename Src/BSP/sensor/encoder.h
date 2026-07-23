#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "config.h"

void encoder_init(void);
void encoder_left_inc(void);
void encoder_right_inc(void);
int  encoder_read(void);

/* 调试：获取上电以来左右轮累计脉冲总数（不清零）。
 * 注意：车速/里程模块使用 encoder_read() 取走并清零增量计数，
 *       但本函数返回的是独立维护的累计总量，不受 encoder_read() 影响。 */
void encoder_get_counts(s32 *left, s32 *right);

/* 调试：清零累计总计数，用于 PPR 测量等场景 */
void encoder_reset_totals(void);

#endif
