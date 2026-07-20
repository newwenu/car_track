#include "ui_debug.h"
#include "ui_dashboard.h"
#include "../app.h"
#include "../../BSP/display/oled_spi.h"
#include "../../BSP/display/oledfont.h"
#include "../control/app_fsm.h"
#include "../trace/trace_control.h"
#include "../obstacle/obstacle_guard.h"
#include "../vehicle/vehicle_state.h"
#include "../mission/lap_counter.h"
#include "../motion/motion_control.h"
#include "../../BSP/actuator/buzzer.h"
#include "../../BSP/actuator/led.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/sensor/encoder.h"
#include "../../BSP/sensor/ultrasonic.h"
#include <stdio.h>

/* 外部报警 LED + 蜂鸣器自检周期 */
#define UI_ACT_LED_PERIOD_TICKS     (500 / UI_TASK_PERIOD_MS)
#define UI_ACT_BEEP_PERIOD_TICKS    (1000 / UI_TASK_PERIOD_MS)

/* 电机自检周期与速度 */
#define UI_MOTOR_PERIOD_TICKS       (2000 / UI_TASK_PERIOD_MS)
#define UI_MOTOR_SPEED_PCT          30

/* 串口打印周期：500ms 输出一次，避免 9600 波特率下占用过多时间 */
#define UI_SERIAL_PERIOD_TICKS      (500 / UI_TASK_PERIOD_MS)

static ui_debug_page_t s_page = UI_DEBUG_INFO;
static u8  s_active = 0;        /* DEBUG 页是否激活：仅在 enter 后置 1，exit 后清 0 */
static u16 s_act_tick = 0;
static u16 s_motor_tick = 0;
static u8  s_motor_dir_idx = 0;
static u16 s_serial_tick = 0;

static const char *s_motor_dir_name[] = {"FWD", "BWD", "LFT", "RGT"};

static void ui_debug_stop_motor(void);

/* 清空 DEBUG 页面显示区域 */
static void ui_debug_clear(void)
{
    oled_spi_clear();
}

/* 绘制页面顶部标题 */
static void ui_debug_draw_header(void)
{
    char buf[16];
    const char *name[] = {"INFO", "TRACE", "STATE", "ACT", "MOTOR"};

    sprintf(buf, "DBG:%s", name[s_page]);
    oled_spi_show_string(0, 0, (u8 *)buf, 8);
}

/* 综合内部变量页 */
static void ui_debug_draw_info(void)
{
    char buf[32];
    int left = 0;
    int right = 0;
    int target_left = 0;
    int target_right = 0;
    s32 enc_left = 0;
    s32 enc_right = 0;

    sprintf(buf, "E:%+4d D:%3d",
            (int)(trace_control_get_error() * 100.0f),
            (int)obstacle_guard_get_distance());
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "V:%2d L:%d/%d",
            vehicle_get_speed_cm_s(),
            lap_counter_get_laps(), 2);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    motion_get_target(&target_left, &target_right);
    sprintf(buf, "TL:%+4d TR:%+4d", target_left, target_right);
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    motion_get_current(&left, &right);
    sprintf(buf, "ML:%+4d MR:%+4d", left, right);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    sprintf(buf, "FSM:%d", (int)fsm_get_state());
    oled_spi_show_string(0, 5, (u8 *)buf, 8);

    encoder_get_counts(&enc_left, &enc_right);
    sprintf(buf, "EL:%+5ld ER:%+5ld", enc_left, enc_right);
    oled_spi_show_string(0, 6, (u8 *)buf, 8);
}

/* 循迹原始数据页 */
static void ui_debug_draw_trace(void)
{
    char buf[32];
    u16 adc[BSP_TRACE_CH_COUNT];
    u8 lost_cnt = 0;
    u8 recovery_cnt = 0;

    trace_read(adc);
    trace_control_get_lost_info(&lost_cnt, &recovery_cnt);

    sprintf(buf, "A0:%4d A1:%4d", adc[0], adc[1]);
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "A2:%4d A3:%4d", adc[2], adc[3]);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    sprintf(buf, "A4:%4d E:%+4d", adc[4],
            (int)(trace_control_get_error() * 100.0f));
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    sprintf(buf, "LE:%+4d LC:%d RC:%d",
            (int)(trace_control_get_last_error() * 100.0f),
            lost_cnt, recovery_cnt);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);
}

/* 系统状态页 */
static void ui_debug_draw_state(void)
{
    char buf[32];
    s32 enc_left = 0;
    s32 enc_right = 0;
    u16 echo_us = 0;
    u8 us_ok = 0;
    u8 us_listen = 0;

    sprintf(buf, "FSM:%d MOT:%d",
            (int)fsm_get_state(), (int)motion_get_state());
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "LOST:%d TIME:%d",
            trace_control_is_lost(), ui_get_run_seconds());
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    sprintf(buf, "UIMODE:%d DIST:%d",
            (int)ui_get_mode(), (int)vehicle_get_distance_cm());
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    encoder_get_counts(&enc_left, &enc_right);
    sprintf(buf, "EL:%+5ld ER:%+5ld", enc_left, enc_right);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);
    sprintf(buf, "US:%4d O:%d I:%d", echo_us, us_ok, us_listen);
    oled_spi_show_string(0, 5, (u8 *)buf, 8);
}

/* 外部报警 LED + 蜂鸣器自检页 */
static void ui_debug_draw_act(void)
{
    char buf[32];

    if (fsm_get_state() == FSM_STATE_IDLE ||
        fsm_get_state() == FSM_STATE_FINISHED)
    {
        sprintf(buf, "ACT:RUN T:%d", s_act_tick);
    }
    else
    {
        sprintf(buf, "ACT:SKIP S:%d", (int)fsm_get_state());
    }

    oled_spi_show_string(0, 1, (u8 *)buf, 8);
}

/* 运行 LED + 蜂鸣器自检逻辑 */
static void ui_debug_update_act(void)
{
    if (fsm_get_state() != FSM_STATE_IDLE &&
        fsm_get_state() != FSM_STATE_FINISHED)
    {
        led_alarm_off();
        return;
    }

    s_act_tick++;

    if ((s_act_tick % UI_ACT_LED_PERIOD_TICKS) == 0)
    {
        led_alarm_toggle();
    }

    if ((s_act_tick % UI_ACT_BEEP_PERIOD_TICKS) == 0)
    {
        buzzer_beep(1000, 100);
    }
}

/* 停止 LED + 蜂鸣器自检 */
static void ui_debug_stop_act(void)
{
    s_act_tick = 0;
    led_alarm_off();
    buzzer_off();
}

/* 电机自检页 */
static void ui_debug_draw_motor(void)
{
    char buf[32];

    if (ui_debug_motor_active())
    {
        sprintf(buf, "MOT:%s SPD:%d T:%d",
                s_motor_dir_name[s_motor_dir_idx],
                UI_MOTOR_SPEED_PCT, s_motor_tick);
    }
    else
    {
        sprintf(buf, "MOT:SKIP S:%d", (int)fsm_get_state());
    }

    oled_spi_show_string(0, 1, (u8 *)buf, 8);
}

/* 运行电机自检逻辑：每周期切换一个方向 */
static void ui_debug_update_motor(void)
{
    if (!ui_debug_motor_active())
    {
        /* 非激活时不在这里停车，避免与 FSM 启动冲突；
         * 切页/退出时会通过 ui_debug_stop_all() 显式停车 */
        return;
    }

    s_motor_tick++;

    if ((s_motor_tick % UI_MOTOR_PERIOD_TICKS) == 0)
    {
        s_motor_dir_idx = (s_motor_dir_idx + 1) % 4;
    }

    motion_run_dir((motion_dir_t)s_motor_dir_idx, UI_MOTOR_SPEED_PCT);
}

/* 停止电机自检 */
static void ui_debug_stop_motor(void)
{
    s_motor_tick = 0;
    s_motor_dir_idx = 0;
    motion_stop();
}

/* 停止当前页可能正在运行的所有 IO 输出 */
static void ui_debug_stop_all(void)
{
    ui_debug_stop_act();
    ui_debug_stop_motor();
}

void ui_debug_enter(void)
{
    s_active = 1;
    s_page = UI_DEBUG_INFO;
    s_serial_tick = 0;
    ui_debug_stop_all();
    ui_debug_clear();
    ui_debug_draw();
}

void ui_debug_exit(void)
{
    ui_debug_stop_all();
    s_active = 0;
    s_page = UI_DEBUG_INFO;
}

void ui_debug_next(void)
{
    ui_debug_stop_all();
    s_page = (ui_debug_page_t)((s_page + 1) % UI_DEBUG_MAX);
    ui_debug_clear();
    ui_debug_draw();
}

void ui_debug_draw(void)
{
    ui_debug_draw_header();

    switch (s_page)
    {
        case UI_DEBUG_INFO:
            ui_debug_draw_info();
            break;

        case UI_DEBUG_TRACE:
            ui_debug_draw_trace();
            break;

        case UI_DEBUG_STATE:
            ui_debug_draw_state();
            break;

        case UI_DEBUG_ACT:
            ui_debug_draw_act();
            break;

        case UI_DEBUG_MOTOR:
            ui_debug_draw_motor();
            break;

        default:
            break;
    }
}

/* 串口周期性输出关键内部变量，便于 PC 端查看/录波 */
static void ui_debug_serial_print(void)
{
    const char *name[] = {"INFO", "TRACE", "STATE", "ACT", "MOTOR"};
    int target_left = 0, target_right = 0;
    int current_left = 0, current_right = 0;
    s32 enc_left = 0, enc_right = 0;
    u16 adc[BSP_TRACE_CH_COUNT];
    u8 lost_cnt = 0, recovery_cnt = 0;
    u16 echo_us = 0;
    u8 us_ok = 0, us_listen = 0;

    s_serial_tick++;
    if (s_serial_tick < UI_SERIAL_PERIOD_TICKS)
    {
        return;
    }
    s_serial_tick = 0;

    motion_get_target(&target_left, &target_right);
    motion_get_current(&current_left, &current_right);
    trace_read(adc);
    encoder_get_counts(&enc_left, &enc_right);
    trace_control_get_lost_info(&lost_cnt, &recovery_cnt);
    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);

    printf("[DBG:%s] E:%+4d D:%3d V:%2d L:%d "
           "A:%4d,%4d,%4d,%4d,%4d "
           "TL:%+4d TR:%+4d ML:%+4d MR:%+4d "
           "FSM:%d MOT:%d LOST:%d\r\n",
           name[s_page],
           (int)(trace_control_get_error() * 100.0f),
           (int)obstacle_guard_get_distance(),
           vehicle_get_speed_cm_s(),
           lap_counter_get_laps(),
           adc[0], adc[1], adc[2], adc[3], adc[4],
           target_left, target_right,
           current_left, current_right,
           (int)fsm_get_state(),
           (int)motion_get_state(),
           trace_control_is_lost());

    printf("[DBG:%s] EL:%+6ld ER:%+6ld LE:%+4d LC:%d RC:%d "
           "US:%4d O:%d I:%d\r\n",
           name[s_page],
           enc_left, enc_right,
           (int)(trace_control_get_last_error() * 100.0f),
           lost_cnt, recovery_cnt,
           echo_us, us_ok, us_listen);
}

void ui_debug_update(void)
{
    if (!s_active)
    {
        /* 安全开关：未进入 DEBUG 页时不应执行任何调试动作 */
        return;
    }

    if (s_page == UI_DEBUG_ACT)
    {
        ui_debug_update_act();
    }
    else if (s_page == UI_DEBUG_MOTOR)
    {
        ui_debug_update_motor();
    }

    ui_debug_draw();
    ui_debug_serial_print();
}

ui_debug_page_t ui_debug_get_page(void)
{
    return s_page;
}

u8 ui_debug_motor_active(void)
{
    return (s_active &&
            s_page == UI_DEBUG_MOTOR &&
            fsm_get_state() == FSM_STATE_IDLE);
}

u8 ui_debug_is_active(void)
{
    return s_active;
}
