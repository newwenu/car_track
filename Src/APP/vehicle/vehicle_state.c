#include "vehicle_state.h"
#include "../../BSP/sensor/encoder.h"

/* 每总脉冲对应的距离（mm * 100 / pulse）
 * 分母乘 2 是因为 encoder_read() 返回左右轮脉冲之和
 * 使用 314/100 近似 PI */
#define VEHICLE_DIST_PER_PULSE_X100 \
    ((u32)((314UL * VEHICLE_WHEEL_DIAMETER_MM) / (VEHICLE_ENCODER_PPR * 2)))

/* 静态变量 */
static u32 s_distance_cm = 0;
static u16 s_speed_cm_s = 0;

void vehicle_init(void)
{
    s_distance_cm = 0;
    s_speed_cm_s = 0;
    encoder_read();  /* 清空编码器初始化期间可能积累的噪声脉冲 */
}

void vehicle_update(void)
{
    int pulses = encoder_read();  /* 取上次调用至今的累计脉冲 */
    u32 dist_inc_x100;

    /* 过滤噪声：负值（方向反转，当前硬件不支持）或孤立脉冲一律丢弃。
     * 阈值 2 意味着 100ms 采样周期内至少需要 2 个有效脉冲，
     * 配合 ISR 消抖可彻底消除振动产生的虚假里程。 */
    if (pulses < 2)
    {
        pulses = 0;
    }

    /* 距离增量：mm * 100 */
    dist_inc_x100 = (u32)pulses * VEHICLE_DIST_PER_PULSE_X100;

    /* 累计里程（cm）：dist_inc_x100 单位是 mm*100，需 /1000 得到 cm */
    s_distance_cm += dist_inc_x100 / 1000;

    /* 速度 = 距离(cm) / 时间(s) = pulses * dist_per_pulse_x100 / sample_ms */
    s_speed_cm_s = (u16)((u32)pulses * VEHICLE_DIST_PER_PULSE_X100 / VEHICLE_SAMPLE_MS);
}

u16 vehicle_get_speed_cm_s(void)
{
    return s_speed_cm_s;
}

u32 vehicle_get_distance_cm(void)
{
    return s_distance_cm;
}

void vehicle_reset_distance(void)
{
    s_distance_cm = 0;
}
