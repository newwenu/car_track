#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "stm32f10x.h"
#include "config.h"

#define I2C_SCL_H  GPIO_SetBits(BSP_OLED_SCL_PORT, BSP_OLED_SCL_PIN)
#define I2C_SCL_L  GPIO_ResetBits(BSP_OLED_SCL_PORT, BSP_OLED_SCL_PIN)
#define I2C_SDA_H  GPIO_SetBits(BSP_OLED_SDA_PORT, BSP_OLED_SDA_PIN)
#define I2C_SDA_L  GPIO_ResetBits(BSP_OLED_SDA_PORT, BSP_OLED_SDA_PIN)
#define I2C_SDA_RD GPIO_ReadInputDataBit(BSP_OLED_SDA_PORT, BSP_OLED_SDA_PIN)

void soft_i2c_init(void);
void soft_i2c_start(void);
void soft_i2c_stop(void);
void soft_i2c_send_byte(u8 data);
u8   soft_i2c_wait_ack(void);
void soft_i2c_write(u8 addr, u8 ctrl, u8 data);

#endif
