#include "soft_i2c.h"
#include "delay.h"

void soft_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(BSP_OLED_CLK_ALL, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_OLED_SCL_PIN | BSP_OLED_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_OLED_SCL_PORT, &GPIO_InitStructure);

    I2C_SCL_H;
    I2C_SDA_H;
}

static void soft_i2c_delay(void)
{
    delay_us(2);
}

void soft_i2c_start(void)
{
    I2C_SDA_H;
    I2C_SCL_H;
    soft_i2c_delay();
    I2C_SDA_L;
    soft_i2c_delay();
    I2C_SCL_L;
}

void soft_i2c_stop(void)
{
    I2C_SDA_L;
    I2C_SCL_H;
    soft_i2c_delay();
    I2C_SDA_H;
    soft_i2c_delay();
}

void soft_i2c_send_byte(u8 data)
{
    u8 i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            I2C_SDA_H;
        else
            I2C_SDA_L;
        data <<= 1;
        soft_i2c_delay();
        I2C_SCL_H;
        soft_i2c_delay();
        I2C_SCL_L;
    }
}

u8 soft_i2c_wait_ack(void)
{
    u8 ack;
    I2C_SDA_H;
    soft_i2c_delay();
    I2C_SCL_H;
    soft_i2c_delay();
    ack = I2C_SDA_RD;
    I2C_SCL_L;
    return ack;
}

void soft_i2c_write(u8 addr, u8 ctrl, u8 data)
{
    soft_i2c_start();
    soft_i2c_send_byte(addr);
    soft_i2c_wait_ack();
    soft_i2c_send_byte(ctrl);
    soft_i2c_wait_ack();
    soft_i2c_send_byte(data);
    soft_i2c_wait_ack();
    soft_i2c_stop();
}
