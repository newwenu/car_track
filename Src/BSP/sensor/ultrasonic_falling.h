#ifndef __ULTRASONIC_FALLING_H
#define __ULTRASONIC_FALLING_H

#include "stm32f10x.h"
#include "config.h"

/*
 * 超声波驱动 - 下降沿捕获版本 (适配LM393开漏输出+上拉电阻)
 *
 * 硬件假设：
 * - 超声波模块使用 LM393 比较器 (开漏输出)
 * - Echo 引脚外接上拉电阻 (如 R23=10KΩ 到 3.3V)
 * - Echo 信号特性：
 *   · 空闲状态: HIGH (被上拉电阻拉高)
 *   · 发射后: 保持 HIGH (模块内部计时中)
 *   · 收到回波: 变 LOW (LM393 比较器输出拉地)
 *   · 低电平持续时间 = 超声波往返时间
 *
 * 与原版 ultrasonic.c 的区别：
 * - GPIO模式: IN_FLOATING (原版: IPD) → 避免与外部上拉冲突
 * - 捕获极性: Falling (原版: Rising) → 匹配Echo下降沿时序
 *
 * 使用方法：
 * 1. 将此文件的函数声明替换原 ultrasonic.h 中的对应声明
 * 或
 * 2. 在 config.h 中通过宏切换使用哪个版本
 */

void  ultrasonic_falling_init(void);
void  ultrasonic_falling_start(void);
float ultrasonic_falling_get_distance(void);
float ultrasonic_falling_measure_blocking(void);

/* 调试：获取原始回波时间、完成标志和监听状态 */
void ultrasonic_falling_get_raw(u16 *echo_us, u8 *ok, u8 *listening);

/* 测距完成回调，应用层可重定义以快速响应（如紧急刹车）。
 * 参数 distance 为本次测距结果，单位 cm；0 表示无效。 */
void ultrasonic_falling_distance_ready_callback(float distance);

#endif
