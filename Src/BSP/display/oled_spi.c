#include "oled_spi.h"
#include "delay.h"
#include "oledfont.h"

/* 硬编码引脚：PB5=CLK, PB4=MOSI, PB3=RES, PA15=DC */
#define OLED_CLK_H   GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define OLED_CLK_L   GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define OLED_MOSI_H  GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define OLED_MOSI_L  GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define OLED_RES_H   GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define OLED_RES_L   GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define OLED_DC_H    GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define OLED_DC_L    GPIO_ResetBits(GPIOA, GPIO_Pin_15)

static void oled_spi_gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    OLED_CLK_L;
    OLED_MOSI_H;
    OLED_RES_H;
    OLED_DC_H;
}

static void oled_spi_send_byte(u8 byte)
{
    u8 i;
    for (i = 0; i < 8; i++)
    {
        if (byte & (0x80 >> i))
            OLED_MOSI_H;
        else
            OLED_MOSI_L;
        OLED_CLK_H;
        OLED_CLK_L;
    }
}

static void oled_spi_write_cmd(u8 cmd)
{
    OLED_DC_L;
    oled_spi_send_byte(cmd);
    OLED_DC_H;
}

static void oled_spi_write_data(u8 dat)
{
    OLED_DC_H;
    oled_spi_send_byte(dat);
    OLED_DC_H;
}

static void oled_spi_set_pos(u8 x, u8 y)
{
    x += 2;
    oled_spi_write_cmd(0xB0 | y);
    oled_spi_write_cmd(0x10 | ((x & 0xF0) >> 4));
    oled_spi_write_cmd(0x00 | (x & 0x0F));
}

void oled_spi_init(void)
{
    u32 i, j;

    oled_spi_gpio_init();

    OLED_RES_L;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < 100; j++);
    OLED_RES_H;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < 1000; j++);

    oled_spi_write_cmd(0xAE);

    oled_spi_write_cmd(0xD5);
    oled_spi_write_cmd(0x80);

    oled_spi_write_cmd(0xA8);
    oled_spi_write_cmd(0x3F);

    oled_spi_write_cmd(0xD3);
    oled_spi_write_cmd(0x00);

    oled_spi_write_cmd(0x40);

    oled_spi_write_cmd(0x8D);
    oled_spi_write_cmd(0x14);

    oled_spi_write_cmd(0xA1);

    oled_spi_write_cmd(0xC8);

    oled_spi_write_cmd(0xDA);
    oled_spi_write_cmd(0x12);

    oled_spi_write_cmd(0x81);
    oled_spi_write_cmd(0xCF);

    oled_spi_write_cmd(0xD9);
    oled_spi_write_cmd(0xF1);

    oled_spi_write_cmd(0xDB);
    oled_spi_write_cmd(0x30);

    oled_spi_write_cmd(0xA4);
    oled_spi_write_cmd(0xA6);

    oled_spi_write_cmd(0xAF);

    oled_spi_clear();
}

void oled_spi_clear(void)
{
    u8 i, j;
    for (j = 0; j < 8; j++)
    {
        oled_spi_set_pos(0, j);
        for (i = 0; i < 132; i++)
            oled_spi_write_data(0x00);
    }
}

void oled_spi_show_char(u8 x, u8 y, u8 chr, u8 size)
{
    u8 c = chr - ' ';
    if (x > 120) { x = 0; y += 2; }
    if (size == 16)
    {
        u8 i;
        oled_spi_set_pos(x, y);
        for (i = 0; i < 8; i++)
            oled_spi_write_data(F8X16[c * 16 + i]);
        oled_spi_set_pos(x, y + 1);
        for (i = 0; i < 8; i++)
            oled_spi_write_data(F8X16[c * 16 + i + 8]);
    }
    else
    {
        u8 i;
        oled_spi_set_pos(x, y);
        for (i = 0; i < 6; i++)
            oled_spi_write_data(F6x8[c][i]);
    }
}

static u32 oled_spi_pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

void oled_spi_show_num(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    u8 t, temp, enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_spi_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                oled_spi_show_char(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else enshow = 1;
        }
        oled_spi_show_char(x + (size / 2) * t, y, temp + '0', size);
    }
}

void oled_spi_show_string(u8 x, u8 y, u8 *str, u8 size)
{
    u8 j = 0;
    while (str[j] != '\0')
    {
        oled_spi_show_char(x, y, str[j], size);
        x += 8;
        if (x > 120) { x = 0; y += 2; }
        j++;
    }
}

void oled_spi_show_chinese(u8 x, u8 y, u8 no)
{
    u8 t;
    oled_spi_set_pos(x, y);
    for (t = 0; t < 16; t++)
        oled_spi_write_data(Hzk[2 * no][t]);
    oled_spi_set_pos(x, y + 1);
    for (t = 0; t < 16; t++)
        oled_spi_write_data(Hzk[2 * no + 1][t]);
}

void oled_spi_draw_bmp(u8 x0, u8 y0, u8 x1, u8 y1, u8 bmp[])
{
    u32 j = 0;
    u8 x, y;
    for (y = y0; y < y1; y++)
    {
        oled_spi_set_pos(x0, y);
        for (x = x0; x < x1; x++)
            oled_spi_write_data(bmp[j++]);
    }
}
