#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "config.h"
#include "board.h"

#define KEY_START  PAin(BSP_KEY_START_BIT)
#define KEY_EXT    PBin(BSP_KEY_EXT_BIT)

void key_init(void);
u8   key_start_scan(void);
u8   key_ext_scan(void);

/* 模式键（KEY_START / PA5）中断支持 */
void key_start_irq_init(void);
void key_start_irq_handler(void);  /* 由 APP 层实现 */

#endif
