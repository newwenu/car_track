#ifndef __VEHICLE_STATE_H
#define __VEHICLE_STATE_H

#include "stm32f10x.h"

/* 物理参数：根据实际小车修改 */
#define VEHICLE_WHEEL_DIAMETER_MM   65  /* 驱动轮直径，单位 mm */
#define VEHICLE_ENCODER_PPR         20  /* 编码器每转脉冲数（单轮） */

/* 采样周期（ms），应与调用 vehicle_update() 的周期一致 */
#define VEHICLE_SAMPLE_MS           100

/* 初始化车速/里程状态 */
void vehicle_init(void);

/* 周期性读取编码器并更新速度、里程；建议每 VEHICLE_SAMPLE_MS 调用一次 */
void vehicle_update(void);

/* 获取当前速度，单位 cm/s */
u16 vehicle_get_speed_cm_s(void);

/* 获取累计单次里程，单位 cm */
u32 vehicle_get_distance_cm(void);

/* 重置累计里程 */
void vehicle_reset_distance(void);

#endif
