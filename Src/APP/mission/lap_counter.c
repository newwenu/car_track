#include "lap_counter.h"
#include "../trace/trace_control.h"

/* ===================== 可调参数 ===================== */
#define APOINT_DEBOUNCE         2       /* 2 次连续全黑才认为过 A 点 */
#define LAP_TARGET              2       /* 目标圈数 */

/* ===================== 静态变量 ===================== */
static u8 s_debounce = 0;
static u8 s_a_left = 0;            /* 已离开 A 点标志 */
static u8 s_lap_count = 0;
static u8 s_just_passed = 0;

/* ===================== 接口实现 ===================== */

void lap_counter_init(void)
{
    s_debounce = 0;
    s_a_left = 0;
    s_lap_count = 0;
    s_just_passed = 0;
}

void lap_counter_update(void)
{
    s_just_passed = 0;

    if (!trace_control_is_all_black())
    {
        s_debounce = 0;
        s_a_left = 1;
        return;
    }

    if (s_debounce < APOINT_DEBOUNCE)
    {
        s_debounce++;
        return;
    }

    /* 消抖完成，但仍需确认已离开过 A 点才算新一圈 */
    if (!s_a_left)
    {
        return;
    }

    s_a_left = 0;
    s_lap_count++;
    s_just_passed = 1;
}

u8 lap_counter_just_passed_a(void)
{
    return s_just_passed;
}

u8 lap_counter_get_laps(void)
{
    return s_lap_count;
}

u8 lap_counter_is_finished(void)
{
    return (s_lap_count >= LAP_TARGET);
}

u8 lap_counter_get_target(void)
{
    return LAP_TARGET;
}

void lap_counter_reset(void)
{
    s_debounce = 0;
    s_a_left = 0;
    s_lap_count = 0;
    s_just_passed = 0;
}
