#ifndef __MIC_H
#define __MIC_H

#include "stm32f10x.h"
#include "config.h"
#include "board.h"

/* 咪头检测引脚电平读取宏 */
#define MIC_DET    PBin(BSP_MIC_BIT)

/* 初始化咪头检测接口（PB7 GPIO 输入） */
void mic_init(void);

/* 扫描咪头状态（非阻塞消抖）。
 * 返回值：1 = 检测到有效拍手/声控信号，0 = 无信号。
 * 建议调用周期：10~20ms */
u8   mic_scan(void);

/* 获取原始引脚电平（用于调试观察） */
u8   mic_get_raw(void);

/* 声控功能使能/禁用控制（默认禁用）。
 * 后续 UI 可通过此接口绑定开关。 */
void mic_set_enabled(u8 enabled);
u8   mic_is_enabled(void);

#endif /* __MIC_H */
