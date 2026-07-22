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
#include "../../BSP/input/key.h"
#include <stdio.h>

/* 外部报警 LED + 蜂鸣器自检周期 */
#define UI_ACT_LED_PERIOD_TICKS     (500 / UI_TASK_PERIOD_MS)
#define UI_ACT_BEEP_PERIOD_TICKS    (1000 / UI_TASK_PERIOD_MS)

/* 电机自检周期与速度 */
#define UI_MOTOR_PERIOD_TICKS       (2000 / UI_TASK_PERIOD_MS)
#define UI_MOTOR_SPEED_PCT          80

/* 电机补偿标定参数：固定速度直行 2 秒，根据编码器脉冲差计算建议增益 */
#define UI_MOTOR_CALIB_SPEED_PCT    60
#define UI_MOTOR_CALIB_TICKS        (2000 / UI_TASK_PERIOD_MS)

/* 串口打印周期：500ms 输出一次，避免 9600 波特率下占用过多时间 */
#define UI_SERIAL_PERIOD_TICKS      (500 / UI_TASK_PERIOD_MS)

static ui_debug_page_t s_page = UI_DEBUG_INFO;
static u8  s_active = 0;        /* DEBUG 页是否激活：仅在 enter 后置 1，exit 后清 0 */
static u16 s_act_tick = 0;
static u16 s_motor_tick = 0;
static u8  s_motor_dir_idx = 0;
static u16 s_serial_tick = 0;

/* 电机补偿标定状态 */
static u8  s_motor_calib_state = 0;     /* 0=空闲, 1=运行中, 2=完成 */
static u16 s_motor_calib_tick = 0;
static s32 s_calib_enc_left_start = 0;
static s32 s_calib_enc_right_start = 0;
static s32 s_calib_enc_left = 0;
static s32 s_calib_enc_right = 0;
static float s_calib_left_gain = 1.0f;
static float s_calib_right_gain = 1.0f;

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
    const char *name[] = {"INFO", "TRACE", "STATE", "ACT", "MOTOR", "USONIC"};

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
    sprintf(buf, "ENC:%+5ld/%+5ld", enc_left, enc_right);
    oled_spi_show_string(0, 6, (u8 *)buf, 8);
}

/* 循迹原始数据页 */
static void ui_debug_draw_trace(void)
{
    char buf[32];
    u16 adc[BSP_TRACE_CH_COUNT];
    u16 min_val = 0, max_val = 0, th_high = 0, th_low = 0;
    u8 lost_cnt = 0;
    u8 recovery_cnt = 0;
    u8 calib_state = trace_calib_get_state();

    trace_read(adc);
    trace_control_get_lost_info(&lost_cnt, &recovery_cnt);

    sprintf(buf, "A0:%4d A1:%4d", adc[0], adc[1]);
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "A2:%4d A3:%4d", adc[2], adc[3]);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    if (calib_state == 1)
    {
        /* 学习中 */
        sprintf(buf, "CALIB RUNNING");
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "MOVE B/W 2s");
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
    }
    else if (calib_state == 2)
    {
        /* 学习成功：显示通道 0 的阈值作为参考 */
        trace_calib_get_thresholds(0, &th_high, &th_low);
        sprintf(buf, "OK H:%4d L:%4d", th_high, th_low);
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        trace_calib_get_range(0, &min_val, &max_val);
        sprintf(buf, "R:%4d KEY:REDO", max_val - min_val);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
    }
    else if (calib_state == 3)
    {
        /* 学习失败 */
        sprintf(buf, "CALIB FAIL");
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "KEY:REDO");
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
    }
    else
    {
        /* 未学习：显示误差和丢线计数 */
        sprintf(buf, "E:%+4d LE:%+4d",
                (int)(trace_control_get_error() * 100.0f),
                (int)(trace_control_get_last_error() * 100.0f));
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "LC:%d RC:%d", lost_cnt, recovery_cnt);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
    }

    sprintf(buf, "EXT:CALIB");
    oled_spi_show_string(0, 5, (u8 *)buf, 8);
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

    sprintf(buf, "MODE:%d D:%5lu",
            (int)ui_get_mode(), vehicle_get_distance_cm());
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    encoder_get_counts(&enc_left, &enc_right);
    sprintf(buf, "ENC:%+5ld/%+5ld", enc_left, enc_right);
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

    /* 标定结果显示区 */
    if (s_motor_calib_state == 1)
    {
        sprintf(buf, "CALIB RUN %d%%", UI_MOTOR_CALIB_SPEED_PCT);
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "T:%d/%d", s_motor_calib_tick, UI_MOTOR_CALIB_TICKS);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
    }
    else if (s_motor_calib_state == 2)
    {
        sprintf(buf, "L:%ld R:%ld", s_calib_enc_left, s_calib_enc_right);
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "LG:%.2f RG:%.2f", s_calib_left_gain, s_calib_right_gain);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
        oled_spi_show_string(0, 5, (u8 *)"KEY:REDO", 8);
    }
    else
    {
        oled_spi_show_string(0, 3, (u8 *)"KEY EXT:CALIB", 8);
        oled_spi_show_string(0, 4, (u8 *)"HOLD 2s:EXIT", 8);
    }
}

/* 启动电机补偿标定：临时禁用补偿，直行采集编码器脉冲 */
static void ui_debug_start_motor_calib(void)
{
    if (!ui_debug_motor_active())
    {
        return;
    }

    s_motor_calib_state = 1;
    s_motor_calib_tick = 0;
    s_calib_enc_left = 0;
    s_calib_enc_right = 0;

    /* 标定时禁用现有补偿，确保测得的是电机真实差异 */
    motion_set_gains(1.0f, 1.0f);

    /* 清零编码器起始值 */
    encoder_get_counts(&s_calib_enc_left_start, &s_calib_enc_right_start);

    /* 固定速度直行 */
    motion_set_speed(UI_MOTOR_CALIB_SPEED_PCT);
}

/* 结束电机补偿标定：计算并应用建议增益 */
static void ui_debug_finish_motor_calib(void)
{
    s32 left_now = 0, right_now = 0;
    float ratio;

    motion_stop();

    encoder_get_counts(&left_now, &right_now);
    s_calib_enc_left = left_now - s_calib_enc_left_start;
    s_calib_enc_right = right_now - s_calib_enc_right_start;

    /* 计算建议增益：降低较快一侧的输出 */
    s_calib_left_gain = 1.0f;
    s_calib_right_gain = 1.0f;

    if (s_calib_enc_left > 0 && s_calib_enc_right > 0)
    {
        if (s_calib_enc_left > s_calib_enc_right)
        {
            ratio = (float)s_calib_enc_right / (float)s_calib_enc_left;
            s_calib_left_gain = ratio;
        }
        else
        {
            ratio = (float)s_calib_enc_left / (float)s_calib_enc_right;
            s_calib_right_gain = ratio;
        }
    }

    /* 应用标定结果 */
    motion_set_gains(s_calib_left_gain, s_calib_right_gain);

    s_motor_calib_state = 2;
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

    /* 标定状态机优先于自检方向切换 */
    if (s_motor_calib_state == 1)
    {
        s_motor_calib_tick++;
        if (s_motor_calib_tick >= UI_MOTOR_CALIB_TICKS)
        {
            ui_debug_finish_motor_calib();
        }
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
    s_motor_calib_state = 0;
    s_motor_calib_tick = 0;
    motion_stop();
}

/* 超声调试页：显示距离、原始回波、完成/监听标志、引脚电平、上下拉状态 */
static void ui_debug_draw_ultrasonic(void)
{
    char buf[32];
    u16 echo_us = 0;
    u8 us_ok = 0;
    u8 us_listen = 0;
    u8 pin_high;

    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);
    pin_high = GPIO_ReadInputDataBit(BSP_US_ECHO_PORT, BSP_US_ECHO_PIN);

    sprintf(buf, "D:%3dcm", (int)obstacle_guard_get_distance());
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "ECHO:%4dus", echo_us);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    sprintf(buf, "OK:%d LSTN:%d", us_ok, us_listen);
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    sprintf(buf, "PIN:%s PULL:%s",
            pin_high ? "H" : "L",
            ultrasonic_get_pull() ? "UP" : "DN");
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    sprintf(buf, "KEY:TOGGLE PULL");
    oled_spi_show_string(0, 6, (u8 *)buf, 8);
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

        case UI_DEBUG_ULTRASONIC:
            ui_debug_draw_ultrasonic();
            break;

        default:
            break;
    }
}

/* 串口周期性输出关键内部变量，便于 PC 端查看/录波 */
static void ui_debug_serial_print(void)
{
    const char *name[] = {"INFO", "TRACE", "STATE", "ACT", "MOTOR", "USONIC"};
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
           "A:%4d,%4d,%4d,%4d "
           "TL:%+4d TR:%+4d ML:%+4d MR:%+4d "
           "FSM:%d MOT:%d LOST:%d\r\n",
           name[s_page],
           (int)(trace_control_get_error() * 100.0f),
           (int)obstacle_guard_get_distance(),
           vehicle_get_speed_cm_s(),
           lap_counter_get_laps(),
           adc[0], adc[1], adc[2], adc[3],
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
    else if (s_page == UI_DEBUG_TRACE)
    {
        /* TRACE 页面下 KEY_EXT 短按触发/重新触发阈值学习 */
        if (key_ext_scan())
        {
            trace_calib_start();
        }
    }
    else if (s_page == UI_DEBUG_MOTOR)
    {
        /* MOTOR 页面下 KEY_EXT 短按触发/重新触发补偿标定 */
        if (key_ext_scan())
        {
            if (s_motor_calib_state == 0 || s_motor_calib_state == 2)
            {
                ui_debug_start_motor_calib();
            }
            else if (s_motor_calib_state == 1)
            {
                /* 运行中再次按下：提前结束并计算 */
                ui_debug_finish_motor_calib();
            }
        }
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

/* 超声调试页：短按切换回波引脚上下拉 */
void ui_debug_toggle_ultrasonic_pull(void)
{
    if (!s_active || s_page != UI_DEBUG_ULTRASONIC)
    {
        return;
    }
    ultrasonic_set_pull(!ultrasonic_get_pull());
}
