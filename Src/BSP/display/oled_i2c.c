#include "oled_i2c.h"
#include "soft_i2c.h"
#include "delay.h"
#include "oledfont.h"

static void oled_write_byte(u8 dat, u8 cmd)
{
    soft_i2c_write(BSP_OLED_I2C_ADDR, cmd, dat);
}

static void oled_set_pos(u8 x, u8 y)
{
    oled_write_byte(0xb0 + y, OLED_CMD);
    oled_write_byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    oled_write_byte(x & 0x0f, OLED_CMD);
}

void oled_init(void)
{
    soft_i2c_init();
    delay_ms(800);

    oled_write_byte(0xAE, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0x40, OLED_CMD);
    oled_write_byte(0xB0, OLED_CMD);
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0xFF, OLED_CMD);
    oled_write_byte(0xA1, OLED_CMD);
    oled_write_byte(0xA6, OLED_CMD);
    oled_write_byte(0xA8, OLED_CMD);
    oled_write_byte(0x3F, OLED_CMD);
    oled_write_byte(0xC8, OLED_CMD);
    oled_write_byte(0xD3, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0xD5, OLED_CMD);
    oled_write_byte(0x80, OLED_CMD);
    oled_write_byte(0xD8, OLED_CMD);
    oled_write_byte(0x05, OLED_CMD);
    oled_write_byte(0xD9, OLED_CMD);
    oled_write_byte(0xF1, OLED_CMD);
    oled_write_byte(0xDA, OLED_CMD);
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xDB, OLED_CMD);
    oled_write_byte(0x30, OLED_CMD);
    oled_write_byte(0x8D, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xAF, OLED_CMD);
}

void oled_clear(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
    {
        oled_write_byte(0xb0 + i, OLED_CMD);
        oled_write_byte(0x00, OLED_CMD);
        oled_write_byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            oled_write_byte(0, OLED_DATA);
    }
}

void oled_show_char(u8 x, u8 y, u8 chr, u8 size)
{
    u8 c = chr - ' ';
    if (x > 120) { x = 0; y += 2; }
    if (size == 16)
    {
        u8 i;
        oled_set_pos(x, y);
        for (i = 0; i < 8; i++)
            oled_write_byte(F8X16[c * 16 + i], OLED_DATA);
        oled_set_pos(x, y + 1);
        for (i = 0; i < 8; i++)
            oled_write_byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        u8 i;
        oled_set_pos(x, y);
        for (i = 0; i < 6; i++)
            oled_write_byte(F6x8[c][i], OLED_DATA);
    }
}

static u32 oled_pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

void oled_show_num(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    u8 t, temp, enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                oled_show_char(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else enshow = 1;
        }
        oled_show_char(x + (size / 2) * t, y, temp + '0', size);
    }
}

void oled_show_string(u8 x, u8 y, u8 *str, u8 size)
{
    u8 j = 0;
    while (str[j] != '\0')
    {
        oled_show_char(x, y, str[j], size);
        x += 8;
        if (x > 120) { x = 0; y += 2; }
        j++;
    }
}

void oled_show_chinese(u8 x, u8 y, u8 no)
{
    u8 t;
    oled_set_pos(x, y);
    for (t = 0; t < 16; t++)
        oled_write_byte(Hzk[2 * no][t], OLED_DATA);
    oled_set_pos(x, y + 1);
    for (t = 0; t < 16; t++)
        oled_write_byte(Hzk[2 * no + 1][t], OLED_DATA);
}

void oled_draw_bmp(u8 x0, u8 y0, u8 x1, u8 y1, u8 bmp[])
{
    u32 j = 0;
    u8 x, y;
    for (y = y0; y < y1; y++)
    {
        oled_set_pos(x0, y);
        for (x = x0; x < x1; x++)
            oled_write_byte(bmp[j++], OLED_DATA);
    }
}
