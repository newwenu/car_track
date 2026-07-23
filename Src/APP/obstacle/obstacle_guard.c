#include "obstacle_guard.h"
#include "../../BSP/sensor/ultrasonic.h"

/* ===================== 可调参数 ===================== */
/* 触发/解除距离。
 * 按电机参数，小车带载最大速度约 40~55cm/s，50ms 反应距离约 2~2.5cm，
 * 能耗制动距离通常 <3cm，总停车距离 <5cm。因此触发距离 20cm 偏保守。
 * 若现场验证制动可靠，可降至 12~15cm，让小车更贴近障碍通过。 */
#define AVOID_DIST_CM           20      /* 小于此距离进入避障（cm） */
#define AVOID_CLEAR_CM          35      /* 大于此距离且保持一段时间解除避障（cm） */
#define AVOID_TRIGGER_DEBOUNCE  2       /* 2 * 10ms = 20ms 确认触发 */
#define AVOID_CLEAR_DEBOUNCE    3       /* 3 * 10ms = 30ms 确认解除 */

/* ===================== 静态变量 ===================== */
static float s_distance = 0.0f;
static u8    s_trigger_cnt = 0;
static u8    s_clear_cnt = 0;
static u8    s_triggered = 0;
static volatile u8 s_emergency_request = 0;

/* ===================== 接口实现 ===================== */

void obstacle_guard_init(void)
{
    s_distance = 0.0f;
    s_trigger_cnt = 0;
    s_clear_cnt = 0;
    s_triggered = 0;
}

void obstacle_guard_update(void)
{
    s_distance = ultrasonic_get_distance();

    if (s_distance > 0.0f && s_distance < AVOID_DIST_CM)
    {
        if (s_trigger_cnt < AVOID_TRIGGER_DEBOUNCE)
        {
            s_trigger_cnt++;
        }
        s_clear_cnt = 0;
    }
    else if (s_distance > AVOID_CLEAR_CM)
    {
        if (s_clear_cnt < AVOID_CLEAR_DEBOUNCE)
        {
            s_clear_cnt++;
        }
        s_trigger_cnt = 0;
    }
    else
    {
        /* 在 hysteresis 区间内（20~35cm）：
         * - 不增加 trigger_cnt（避免误触发）
         * - 但允许 clear_cnt 累积（允许渐进式解除）
         * - 这是修复"无法解除"问题的关键改动 */
        if (s_triggered == 1)
        {
            if (s_clear_cnt < AVOID_CLEAR_DEBOUNCE)
            {
                s_clear_cnt++;
            }
        }
        s_trigger_cnt = 0;
    }

    if (s_trigger_cnt >= AVOID_TRIGGER_DEBOUNCE)
    {
        s_triggered = 1;
    }
    else if (s_clear_cnt >= AVOID_CLEAR_DEBOUNCE)
    {
        s_triggered = 0;
    }
}

u8 obstacle_guard_is_triggered(void)
{
    return s_triggered;
}

u8 obstacle_guard_is_cleared(void)
{
    return (s_triggered == 0) && (s_clear_cnt >= AVOID_CLEAR_DEBOUNCE);
}

float obstacle_guard_get_distance(void)
{
    return s_distance;
}

void obstacle_guard_set_emergency_request(void)
{
    s_emergency_request = 1;
}

u8 obstacle_guard_has_emergency_request(void)
{
    return s_emergency_request;
}

void obstacle_guard_clear_emergency_request(void)
{
    s_emergency_request = 0;
}

/* 超声波测距完成回调：距离过近时立即请求刹车。
 * 该强定义会覆盖 BSP 层 ultrasonic.c 中的弱空实现。 */
void ultrasonic_distance_ready_callback(float distance)
{
    if (distance > 0.0f && distance < AVOID_DIST_CM)
    {
        obstacle_guard_set_emergency_request();
    }
}
