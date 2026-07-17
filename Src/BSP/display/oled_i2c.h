#ifndef __OLED_I2C_H
#define __OLED_I2C_H

#include "stm32f10x.h"
#include "config.h"

#define OLED_CMD  0x00
#define OLED_DATA 0x40

void oled_init(void);
void oled_clear(void);
void oled_show_char(u8 x, u8 y, u8 chr, u8 size);
void oled_show_string(u8 x, u8 y, u8 *str, u8 size);
void oled_show_num(u8 x, u8 y, u32 num, u8 len, u8 size);
void oled_show_chinese(u8 x, u8 y, u8 no);
void oled_draw_bmp(u8 x0, u8 y0, u8 x1, u8 y1, u8 bmp[]);

#endif
