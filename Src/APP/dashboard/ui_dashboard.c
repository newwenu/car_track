#include "ui_dashboard.h"
#include "../../BSP/display/oled_spi.h"
#include "../../BSP/display/oledfont.h"
#include "../../BSP/input/key.h"

/* Hzk[] 数组中文字索引（与 oledfont.h 中的顺序对应） */
#define UI_CH_SHE    0  /* 设 */
#define UI_CH_DING   1  /* 定 */
#define UI_CH_CHE    2  /* 车 */
#define UI_CH_SU     3  /* 速 */
#define UI_CH_ZUO    4  /* 左 */
#define UI_CH_YOU    5  /* 右 */
#define UI_CH_LUN    6  /* 轮 */
#define UI_CH_DU     7  /* 度 */
#define UI_CH_LI     8  /* 里 */
#define UI_CH_CHENG  9  /* 程 */
#define UI_CH_MO    10  /* 模 */
#define UI_CH_SHI   11  /* 式 */
#define UI_CH_SHI2  12  /* 时 */
#define UI_CH_JIAN  13  /* 间 */
#define UI_CH_GUI   14  /* 规 */
#define UI_CH_ZHENG 15  /* 正 */
#define UI_CH_CHANG 16  /* 常 */

/* 无操作超时时间（单位：ms） */
#define UI_TIMEOUT_MS       3000
#define UI_TIMEOUT_TICKS    (UI_TIMEOUT_MS / UI_TASK_PERIOD_MS)

/* 按键消抖次数（连续 N 次采样一致才确认状态） */
#define UI_KEY_DEBOUNCE     2

/* 计时器：每 1s 增加一次（UI_TASK_PERIOD_MS * UI_TIMER_TICKS = 1000ms） */
#define UI_TIMER_TICKS      (1000 / UI_TASK_PERIOD_MS)

/* 静态变量 */
static ui_page_t s_current_page = UI_PAGE_MAIN;
static u16 s_speed = 0;
static u32 s_mileage = 0;
static ui_mode_t s_mode = UI_MODE_NORMAL;
static volatile u8 s_key_irq_flag = 0;  /* 模式键中断标志，由 ISR 设置 */
static u16 s_run_seconds = 0;           /* 运行计时（秒） */
static u16 s_timer_count = 0;           /* 计时分频计数 */
static u8  s_timer_running = 1;         /* 计时器运行标志 */

/* 在指定位置连续显示多个汉字（16x16） */
static void ui_draw_chinese_string(u8 x, u8 y, const u8 *idx, u8 len)
{
    u8 i;
    for (i = 0; i < len; i++)
    {
        oled_spi_show_chinese(x + i * 16, y, idx[i]);
    }
}

/* 主界面静态元素：速度 + 时间 + 里程 */
static void ui_draw_main_static(void)
{
    const u8 title_speed[]   = {UI_CH_SU, UI_CH_DU};     /* 速度 */
    const u8 title_time[]    = {UI_CH_SHI2, UI_CH_JIAN}; /* 时间 */
    const u8 title_mileage[] = {UI_CH_LI, UI_CH_CHENG};  /* 里程 */

    ui_draw_chinese_string(0, 0, title_speed, 2);
    oled_spi_show_string(48, 0, (u8 *)"cm/s", 16);

    ui_draw_chinese_string(0, 2, title_time, 2);

    ui_draw_chinese_string(0, 4, title_mileage, 2);
    oled_spi_show_string(72, 4, (u8 *)"cm", 16);
}

/* 模式界面静态元素 */
static void ui_draw_mode_static(void)
{
    const u8 title_mode[] = {UI_CH_MO, UI_CH_SHI};  /* 模式 */
    ui_draw_chinese_string(0, 0, title_mode, 2);
}

/* 绘制速度数值（两位数字，x=32，y=0） */
static void ui_draw_speed(void)
{
    oled_spi_show_num(32, 0, s_speed, 2, 16);
}

/* 绘制时间数值（MM:SS，x=32，y=2） */
static void ui_draw_time(void)
{
    u8 buf[6];
    u8 min = s_run_seconds / 60;
    u8 sec = s_run_seconds % 60;

    buf[0] = '0' + (min / 10);
    buf[1] = '0' + (min % 10);
    buf[2] = ':';
    buf[3] = '0' + (sec / 10);
    buf[4] = '0' + (sec % 10);
    buf[5] = '\0';

    oled_spi_show_string(32, 2, buf, 16);
}

/* 绘制里程数值（五位数字，x=32，y=4） */
static void ui_draw_mileage(void)
{
    oled_spi_show_num(32, 4, s_mileage, 5, 16);
}

/* 绘制模式字段（正常模式 / 规定模式） */
static void ui_draw_mode(void)
{
    const u8 mode_normal[] = {UI_CH_ZHENG, UI_CH_CHANG, UI_CH_MO, UI_CH_SHI}; /* 正常模式 */
    const u8 mode_rule[]   = {UI_CH_GUI, UI_CH_DING, UI_CH_MO, UI_CH_SHI};     /* 规定模式 */

    /* 先清空预留区域，防止切换时残留 */
    oled_spi_show_string(32, 0, (u8 *)"        ", 16);

    if (s_mode == UI_MODE_NORMAL)
    {
        ui_draw_chinese_string(32, 0, mode_normal, 4);
    }
    else
    {
        ui_draw_chinese_string(32, 0, mode_rule, 4);
    }
}

void ui_init(void)
{
    oled_spi_init();
    oled_spi_clear();
    key_start_irq_init();
    ui_switch_page(UI_PAGE_MAIN);
}

void ui_switch_page(ui_page_t page)
{
    if (page >= UI_PAGE_MAX)
    {
        return;
    }

    s_current_page = page;
    oled_spi_clear();

    if (page == UI_PAGE_MAIN)
    {
        ui_draw_main_static();
        ui_draw_speed();
        ui_draw_time();
        ui_draw_mileage();
    }
    else
    {
        ui_draw_mode_static();
        ui_draw_mode();
    }
}

void ui_set_speed(u16 speed)
{
    if (speed > 99)
    {
        speed = 99;
    }

    if (s_speed != speed)
    {
        s_speed = speed;
        if (s_current_page == UI_PAGE_MAIN)
        {
            ui_draw_speed();
        }
    }
}

void ui_set_mileage(u32 mileage)
{
    if (mileage > 99999)
    {
        mileage = 99999;
    }

    if (s_mileage != mileage)
    {
        s_mileage = mileage;
        if (s_current_page == UI_PAGE_MAIN)
        {
            ui_draw_mileage();
        }
    }
}

void ui_set_mode(ui_mode_t mode)
{
    if (mode >= UI_MODE_MAX)
    {
        return;
    }

    if (s_mode != mode)
    {
        s_mode = mode;
        if (s_current_page == UI_PAGE_MODE)
        {
            ui_draw_mode();
        }
    }
}

void ui_refresh(void)
{
    if (s_current_page == UI_PAGE_MAIN)
    {
        ui_draw_speed();
        ui_draw_time();
        ui_draw_mileage();
    }
    else
    {
        ui_draw_mode();
    }
}

/* 计时器控制 */
void ui_timer_start(void)
{
    s_timer_running = 1;
}

void ui_timer_stop(void)
{
    s_timer_running = 0;
}

void ui_timer_reset(void)
{
    s_run_seconds = 0;
    s_timer_count = 0;
    if (s_current_page == UI_PAGE_MAIN)
    {
        ui_draw_time();
    }
}

/* 模式键中断回调：设置标志并临时屏蔽该中断线，防止抖动反复进入 */
void key_start_irq_handler(void)
{
    s_key_irq_flag = 1;
    EXTI->IMR &= ~EXTI_Line5;
}

/* 轮询按键并处理界面/模式切换；建议每 UI_TASK_PERIOD_MS 调用一次 */
void ui_task(void)
{
    static u16 idle_ticks = 0;
    static u8 debounce_cnt = 0;
    static u8 key_state = 0;  /* 0:空闲 1:消抖中 2:等待释放 */
    u8 key_pressed = 0;

    /* 计时器累加 */
    if (s_timer_running)
    {
        s_timer_count++;
        if (s_timer_count >= UI_TIMER_TICKS)
        {
            s_timer_count = 0;
            if (s_run_seconds < 5999)  /* 最大 99:59 */
            {
                s_run_seconds++;
            }
            if (s_current_page == UI_PAGE_MAIN)
            {
                ui_draw_time();
            }
        }
    }

    /* 中断触发后开始消抖 */
    if (s_key_irq_flag && key_state == 0)
    {
        s_key_irq_flag = 0;
        key_state = 1;
        debounce_cnt = 0;
    }

    if (key_state == 1)
    {
        debounce_cnt++;
        if (debounce_cnt >= UI_KEY_DEBOUNCE)
        {
            key_state = 2;
            if (KEY_START == 0)  /* 消抖后仍为按下 */
            {
                key_pressed = 1;
            }
        }
    }
    else if (key_state == 2)
    {
        if (KEY_START == 1)  /* 已释放，重新开放中断 */
        {
            key_state = 0;
            EXTI->IMR |= EXTI_Line5;
        }
    }

    if (key_pressed)
    {
        idle_ticks = 0;

        if (s_current_page == UI_PAGE_MAIN)
        {
            /* 在主界面按模式键：进入模式选择界面 */
            ui_switch_page(UI_PAGE_MODE);
        }
        else
        {
            /* 在模式界面按模式键：循环切换模式 */
            ui_mode_t next_mode = (ui_mode_t)((s_mode + 1) % UI_MODE_MAX);
            ui_set_mode(next_mode);
        }
    }
    else if (s_current_page == UI_PAGE_MODE)
    {
        /* 模式界面下无操作，超时后返回主界面 */
        idle_ticks++;
        if (idle_ticks >= UI_TIMEOUT_TICKS)
        {
            idle_ticks = 0;
            ui_switch_page(UI_PAGE_MAIN);
        }
    }
    else
    {
        /* 主界面下重置空闲计数 */
        idle_ticks = 0;
    }
}
