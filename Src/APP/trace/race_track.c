#include "race_track.h"
#include "trace_control.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/system/config.h"
#include <stddef.h>

#define RT_BASE_SPEED_MAX        40
#define RT_BASE_SPEED_NORMAL     36
#define RT_BASE_SPEED_LEFT_SIDE  38
#define RT_BASE_SPEED_CURVE      28
#define RT_BASE_SPEED_SHARP      20
#define RT_BASE_SPEED_RIGHT_ANGLE 14
#define RT_BASE_SPEED_WAVE       26
#define RT_BASE_SPEED_CROSS      22
#define RT_BASE_SPEED_CONCAVE    16
#define RT_BASE_SPEED_WAVE_ENTRY 30

#define RT_KP_SMALL              34
#define RT_KP_MEDIUM             26
#define RT_KP_LARGE              18
#define RT_KP_RIGHT_ANGLE        40
#define RT_KP_CONCAVE_LEFT       45
#define RT_KD                    14
#define RT_KD_RIGHT_ANGLE        20
#define RT_KD_CONCAVE            24
#define RT_MAX_STEER             40
#define RT_MAX_STEER_RIGHT_ANGLE 46
#define RT_MAX_STEER_CONCAVE     48

#define RT_TH_HIGH               800
#define RT_TH_LOW                500
#define RT_ADC_CENTER            650

#define RT_ERROR_STRAIGHT        0.30f
#define RT_ERROR_CURVE           0.75f
#define RT_ERROR_SHARP           1.10f
#define RT_RIGHT_ANGLE_ACC_THR   0.7f
#define RT_RIGHT_ANGLE_DETECT_TICKS 2
#define RT_CONCAVE_DETECT_THR    1.0f
#define RT_CONCAVE_DURATION      12

#define RT_PREDICT_LOOKAHEAD     3
#define RT_PREDICT_COEF          0.35f
#define RT_ANTI_OUT_GAIN         2.0f
#define RT_EDGE_THRESHOLD        1.1f
#define RT_LEFT_TURN_BOOST       1.15f

#define RT_WAVE_DETECT_WINDOW    10
#define RT_WAVE_SIGN_CHANGE_THR  4
#define RT_WAVE_DIRECTION_CHECK  5

#define RT_LOST_CNT_THR          3
#define RT_LOST_TIMEOUT          220
#define RT_LOST_SEARCH_SPD       26
#define RT_LOST_STEER            16

#define RT_SMOOTH_RATE           0.14f
#define RT_SMOOTH_RATE_RIGHT_ANGLE 0.22f
#define RT_SMOOTH_RATE_CONCAVE    0.25f
#define RT_EXIT_BRAKE_ZONE       0.48f
#define RT_EXIT_BRAKE_COEF       6.5f

#define RT_CROSS_DETECT_TICKS    18
#define RT_CROSS_EXIT_COOLDOWN   12
#define RT_CROSS_STABLE_THR      3
#define RT_CROSS_ADC_SPREAD      200
#define RT_CROSS_CONFUSION_THR   2
#define RT_HISTORY_DEPTH         10
#define RT_RIGHT_ANGLE_COOLDOWN  25
#define RT_CONCAVE_COOLDOWN      18

static const float rt_weights[BSP_TRACE_CH_COUNT] = {
    +1.5f, +0.5f, -0.5f, -1.5f
};

static u16   rt_vals[BSP_TRACE_CH_COUNT];
static u8    rt_black[BSP_TRACE_CH_COUNT];
static float rt_error = 0.0f;
static float rt_last_error = 0.0f;
static float rt_last_last_error = 0.0f;
static float rt_error_hist[RT_HISTORY_DEPTH];
static int   rt_left_pct = 0;
static int   rt_right_pct = 0;
static float rt_current_speed = 36.0f;
static u8    rt_lost_cnt = 0;
static u8    rt_lost_recovery = 0;
static u8    rt_is_lost = 0;

static u8    rt_sign_change_cnt = 0;
static float rt_prev_error = 0.0f;
static u8    rt_is_wave = 0;
static u8    rt_wave_cooldown = 0;

static u8    rt_all_black_recent = 0;
static u8    rt_cross_exit_cooldown = 0;
static float rt_cross_entry_error = 0.0f;
static u8    rt_is_confused = 0;
static u8    rt_confusion_cnt = 0;

static u8    rt_is_right_angle = 0;
static u8    rt_right_angle_cooldown = 0;
static u8    rt_right_angle_detect_cnt = 0;

static u8    rt_is_concave_section = 0;
static u8    rt_concave_cooldown = 0;
static u8    rt_concave_detect_cnt = 0;

static u8    rt_is_left_side = 0;
static u8    rt_left_side_stable_cnt = 0;

static s8    rt_wave_direction = 0;
static s8    rt_wave_dir_check_cnt = 0;

static float rt_adc_error = 0.0f;
static float rt_predicted_error = 0.0f;

static void rt_init_history(void)
{
    u8 i;
    for (i = 0; i < RT_HISTORY_DEPTH; i++)
    {
        rt_error_hist[i] = 0.0f;
    }
}

static void rt_push_history(float err)
{
    u8 i;
    for (i = RT_HISTORY_DEPTH - 1; i > 0; i--)
    {
        rt_error_hist[i] = rt_error_hist[i-1];
    }
    rt_error_hist[0] = err;
}

static void rt_update_black_state(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (rt_black[i])
        {
            if (rt_vals[i] < RT_TH_LOW)
            {
                rt_black[i] = 0;
            }
        }
        else
        {
            if (rt_vals[i] > RT_TH_HIGH)
            {
                rt_black[i] = 1;
            }
        }
    }
}

static u8 rt_check_all_black(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (!rt_black[i]) return 0;
    }
    return 1;
}

static float rt_calc_error(void)
{
    float sum = 0.0f, cnt = 0.0f;
    u8 i;

    rt_update_black_state();

    if (rt_check_all_black())
    {
        rt_all_black_recent = RT_CROSS_DETECT_TICKS;
        rt_cross_entry_error = rt_last_error;
        rt_cross_exit_cooldown = RT_CROSS_EXIT_COOLDOWN;
        rt_is_confused = 0;
        rt_confusion_cnt = 0;
    }

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (rt_black[i])
        {
            sum += rt_weights[i];
            cnt += 1.0f;
        }
    }

    if (cnt > 0.0f)
    {
        rt_lost_cnt = 0;
        rt_lost_recovery = 0;
        rt_is_lost = 0;

        if ((rt_black[0] == 0) && (rt_black[3] == 0) &&
            ((rt_black[1] == 1) || (rt_black[2] == 1)))
        {
            return 0.0f;
        }

        return sum / cnt;
    }

    if (rt_lost_cnt < RT_LOST_CNT_THR)
    {
        rt_lost_cnt++;
    }

    if (rt_lost_cnt >= RT_LOST_CNT_THR)
    {
        rt_is_lost = 1;
    }

    return rt_last_error;
}

static float rt_calc_adc_error(void)
{
    float adc_sum = 0.0f, weight_sum = 0.0f;
    float normalized;
    u8 i;

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        normalized = ((float)rt_vals[i] - RT_ADC_CENTER) / (RT_TH_HIGH - RT_ADC_CENTER);
        if (normalized > 1.0f) normalized = 1.0f;
        if (normalized < 0.0f) normalized = 0.0f;

        adc_sum += rt_weights[i] * normalized;
        weight_sum += normalized;
    }

    if (weight_sum < 0.05f)
    {
        return rt_last_error;
    }

    return adc_sum / weight_sum;
}

static float rt_predict_error(void)
{
    float pred = 0.0f;
    float trend = 0.0f;
    s8 i;

    if (RT_HISTORY_DEPTH < 2) return rt_error;

    for (i = 0; i < RT_PREDICT_LOOKAHEAD && i < RT_HISTORY_DEPTH - 1; i++)
    {
        trend = rt_error_hist[i] - rt_error_hist[i + 1];
        pred += trend * (float)(i + 1);
    }

    pred /= (float)RT_PREDICT_LOOKAHEAD;

    return rt_error + RT_PREDICT_COEF * pred;
}

static float rt_anti_out_enhancement(float error)
{
    float abs_err = (error >= 0.0f) ? error : -error;
    float enhancement = 0.0f;

    if (abs_err > RT_EDGE_THRESHOLD)
    {
        enhancement = (abs_err - RT_EDGE_THRESHOLD) * RT_ANTI_OUT_GAIN;
        if (error > 0.0f)
        {
            return error + enhancement;
        }
        else
        {
            return error - enhancement;
        }
    }

    return error;
}

static void rt_detect_wave(void)
{
    if ((rt_error * rt_prev_error < 0.0f) &&
        (rt_error != 0.0f) && (rt_prev_error != 0.0f))
    {
        rt_sign_change_cnt++;
    }

    if (rt_sign_change_cnt >= RT_WAVE_SIGN_CHANGE_THR)
    {
        rt_is_wave = 1;
        rt_wave_cooldown = RT_WAVE_DETECT_WINDOW;
        rt_sign_change_cnt = 0;
    }

    if (rt_wave_cooldown > 0)
    {
        rt_wave_cooldown--;
    }
    else
    {
        rt_is_wave = 0;
        rt_sign_change_cnt = 0;
    }

    rt_prev_error = rt_error;
}

static u8 rt_detect_right_angle(void)
{
    float error_vel, error_acc;
    float abs_acc;

    if (rt_right_angle_cooldown > 0)
    {
        rt_right_angle_cooldown--;
        if (rt_right_angle_cooldown == 0)
        {
            rt_is_right_angle = 0;
        }
        return rt_is_right_angle;
    }

    error_vel = rt_error - rt_last_error;
    error_acc = error_vel - (rt_last_error - rt_last_last_error);
    abs_acc = (error_acc >= 0.0f) ? error_acc : -error_acc;

    if (abs_acc > RT_RIGHT_ANGLE_ACC_THR)
    {
        rt_right_angle_detect_cnt++;
        if (rt_right_angle_detect_cnt >= RT_RIGHT_ANGLE_DETECT_TICKS)
        {
            rt_is_right_angle = 1;
            rt_right_angle_cooldown = RT_RIGHT_ANGLE_COOLDOWN;
            rt_right_angle_detect_cnt = 0;
            return 1;
        }
    }
    else
    {
        if (rt_right_angle_detect_cnt > 0)
        {
            rt_right_angle_detect_cnt--;
        }
    }

    return rt_is_right_angle;
}

static u8 rt_detect_concave_section(void)
{
    float abs_err;

    if (rt_concave_cooldown > 0)
    {
        rt_concave_cooldown--;
        if (rt_concave_cooldown == 0)
        {
            rt_is_concave_section = 0;
        }
        return rt_is_concave_section;
    }

    abs_err = (rt_error >= 0.0f) ? rt_error : -rt_error;

    if (rt_error < -RT_CONCAVE_DETECT_THR && abs_err > RT_ERROR_CURVE)
    {
        rt_concave_detect_cnt++;
        if (rt_concave_detect_cnt >= 3)
        {
            rt_is_concave_section = 1;
            rt_concave_cooldown = RT_CONCAVE_DURATION;
            rt_concave_detect_cnt = 0;
            return 1;
        }
    }
    else
    {
        if (rt_concave_detect_cnt > 0) rt_concave_detect_cnt--;
    }

    return rt_is_concave_section;
}

static void rt_detect_left_side(void)
{
    float abs_err = (rt_error >= 0.0f) ? rt_error : -rt_error;

    if (abs_err < RT_ERROR_STRAIGHT)
    {
        rt_left_side_stable_cnt++;
        if (rt_left_side_stable_cnt > 15)
        {
            rt_is_left_side = 1;
        }
    }
    else
    {
        rt_left_side_stable_cnt = 0;
        rt_is_left_side = 0;
    }
}

static void rt_analyze_wave_direction(void)
{
    float trend_sum = 0.0f;
    s8 i;

    for (i = 0; i < RT_WAVE_DIRECTION_CHECK && i < RT_HISTORY_DEPTH - 1; i++)
    {
        trend_sum += rt_error_hist[i] - rt_error_hist[i + 1];
    }

    if (trend_sum > 0.5f)
    {
        rt_wave_dir_check_cnt++;
        if (rt_wave_dir_check_cnt >= 3)
        {
            rt_wave_direction = 1;
        }
    }
    else if (trend_sum < -0.5f)
    {
        rt_wave_dir_check_cnt--;
        if (rt_wave_dir_check_cnt <= -3)
        {
            rt_wave_direction = -1;
        }
    }
}

static u8 rt_detect_cross_section(void)
{
    u8 black_cnt = 0;
    u8 i;
    u16 min_adc = 4095, max_adc = 0;
    float adc_spread;

    if (rt_all_black_recent > 0)
    {
        if (rt_cross_exit_cooldown > 0)
        {
            rt_cross_exit_cooldown--;
        }

        rt_all_black_recent--;

        return 1;
    }

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (rt_black[i])
        {
            black_cnt++;
            if (rt_vals[i] < min_adc) min_adc = rt_vals[i];
            if (rt_vals[i] > max_adc) max_adc = rt_vals[i];
        }
    }

    if (black_cnt >= 3)
    {
        adc_spread = (float)(max_adc - min_adc);

        if (black_cnt == 4 || adc_spread < RT_CROSS_ADC_SPREAD)
        {
            rt_cross_entry_error = rt_last_error;
            rt_all_black_recent = RT_CROSS_DETECT_TICKS;
            rt_cross_exit_cooldown = RT_CROSS_EXIT_COOLDOWN;
            rt_is_confused = 0;
            rt_confusion_cnt = 0;

            return 1;
        }
    }

    if (black_cnt == 2)
    {
        u8 gap_present = 0;
        if (rt_black[0] && rt_black[3]) gap_present = 1;
        if (rt_black[0] && rt_black[2] && !rt_black[1]) gap_present = 1;
        if (rt_black[1] && rt_black[3] && !rt_black[2]) gap_present = 1;

        if (gap_present)
        {
            rt_confusion_cnt++;
            if (rt_confusion_cnt >= RT_CROSS_CONFUSION_THR)
            {
                rt_is_confused = 1;
                rt_cross_entry_error = rt_last_error;
                return 1;
            }
        }
        else
        {
            if (rt_confusion_cnt > 0) rt_confusion_cnt--;
        }
    }
    else
    {
        if (rt_confusion_cnt > 0) rt_confusion_cnt--;
    }

    return 0;
}

static float rt_get_cross_error(void)
{
    float use_error;

    if (rt_is_confused)
    {
        use_error = rt_cross_entry_error * 0.7f + rt_error * 0.3f;
    }
    else if (rt_all_black_recent > RT_CROSS_DETECT_TICKS - 5)
    {
        use_error = rt_cross_entry_error * 0.5f + rt_error * 0.5f;
    }
    else
    {
        use_error = rt_cross_entry_error * 0.3f + rt_error * 0.7f;
    }

    return use_error;
}

static float rt_get_adaptive_kp(float error)
{
    float abs_err = (error >= 0.0f) ? error : -error;

    if (abs_err < 0.45f)
    {
        return RT_KP_SMALL;
    }
    else if (abs_err < 0.95f)
    {
        return RT_KP_MEDIUM;
    }
    else
    {
        return RT_KP_LARGE;
    }
}

static int rt_select_base_speed(float error, u8 is_cross, u8 is_right_angle)
{
    float abs_err = (error >= 0.0f) ? error : -error;
    int target_speed;

    rt_detect_left_side();
    rt_detect_concave_section();

    if (rt_is_concave_section)
    {
        target_speed = RT_BASE_SPEED_CONCAVE;
    }
    else if (is_right_angle)
    {
        target_speed = RT_BASE_SPEED_RIGHT_ANGLE;
    }
    else if (rt_is_wave)
    {
        if (rt_wave_direction == 1 && abs_err < RT_ERROR_CURVE)
        {
            target_speed = RT_BASE_SPEED_WAVE_ENTRY;
        }
        else
        {
            target_speed = RT_BASE_SPEED_WAVE;
        }
    }
    else if (is_cross)
    {
        target_speed = RT_BASE_SPEED_CROSS;
    }
    else if (abs_err > RT_ERROR_SHARP)
    {
        target_speed = RT_BASE_SPEED_SHARP;
    }
    else if (abs_err > RT_ERROR_CURVE)
    {
        target_speed = RT_BASE_SPEED_CURVE;
    }
    else if (rt_is_left_side)
    {
        target_speed = RT_BASE_SPEED_LEFT_SIDE;
    }
    else
    {
        target_speed = RT_BASE_SPEED_NORMAL;
    }

    return target_speed;
}

static int rt_smooth_speed(int target)
{
    rt_current_speed += RT_SMOOTH_RATE * ((float)target - rt_current_speed);
    return (int)(rt_current_speed + 0.5f);
}

static float rt_calc_exit_brake(float err, float last_err, float last_last_err)
{
    float abs_err = (err >= 0.0f) ? err : -err;
    float abs_last = (last_err >= 0.0f) ? last_err : -last_err;
    float vel = err - last_err;

    if ((abs_err < RT_EXIT_BRAKE_ZONE) &&
        (abs_err > 0.08f) &&
        (err * last_err > 0.0f) &&
        (vel * err < 0.0f) &&
        (abs_last > RT_EXIT_BRAKE_ZONE))
    {
        float brake = -RT_EXIT_BRAKE_COEF * vel;
        const float max_b = 10.0f;
        if (brake > max_b) brake = max_b;
        if (brake < -max_b) brake = -max_b;
        return brake;
    }

    return 0.0f;
}

void race_track_init(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        rt_vals[i] = 0;
        rt_black[i] = 0;
    }
    rt_error = 0.0f;
    rt_last_error = 0.0f;
    rt_last_last_error = 0.0f;
    rt_left_pct = 0;
    rt_right_pct = 0;
    rt_current_speed = (float)RT_BASE_SPEED_NORMAL;
    rt_lost_cnt = 0;
    rt_lost_recovery = 0;
    rt_is_lost = 0;
    rt_sign_change_cnt = 0;
    rt_prev_error = 0.0f;
    rt_is_wave = 0;
    rt_wave_cooldown = 0;
    rt_all_black_recent = 0;
    rt_cross_exit_cooldown = 0;
    rt_cross_entry_error = 0.0f;
    rt_is_confused = 0;
    rt_confusion_cnt = 0;
    rt_is_right_angle = 0;
    rt_right_angle_cooldown = 0;
    rt_right_angle_detect_cnt = 0;
    rt_is_concave_section = 0;
    rt_concave_cooldown = 0;
    rt_concave_detect_cnt = 0;
    rt_is_left_side = 0;
    rt_left_side_stable_cnt = 0;
    rt_wave_direction = 0;
    rt_wave_dir_check_cnt = 0;
    rt_adc_error = 0.0f;
    rt_predicted_error = 0.0f;

    rt_init_history();
}

void race_track_update(void)
{
    float error, diff;
    int base_speed;
    u8 is_cross, is_right_angle;

    trace_read(rt_vals);

    error = rt_calc_error();
    rt_adc_error = rt_calc_adc_error();
    rt_push_history(error);
    rt_predicted_error = rt_predict_error();

    is_cross = rt_detect_cross_section();

    if (rt_is_lost == 0)
    {
        if (is_cross)
        {
            error = rt_get_cross_error();
        }
        else
        {
            float abs_digital = (error >= 0.0f) ? error : -error;
            float abs_adc = (rt_adc_error >= 0.0f) ? rt_adc_error : -rt_adc_error;
            float abs_pred = (rt_predicted_error >= 0.0f) ? rt_predicted_error : -rt_predicted_error;

            if (abs_adc > 0.1f && (abs_adc < abs_digital || abs_digital < 0.3f))
            {
                error = rt_adc_error * 0.6f + error * 0.4f;
            }

            if (abs_pred > 0.15f && ((error > 0.0f && rt_predicted_error > 0.0f) ||
                                       (error < 0.0f && rt_predicted_error < 0.0f)))
            {
                error = error * 0.7f + rt_predicted_error * 0.3f;
            }
        }

        error = rt_anti_out_enhancement(error);
    }

    rt_error = error;

    rt_detect_wave();
    is_right_angle = rt_detect_right_angle();
    rt_analyze_wave_direction();

    if (rt_is_lost)
    {
        rt_lost_recovery++;
        if (rt_lost_recovery >= RT_LOST_TIMEOUT)
        {
            rt_left_pct = 0;
            rt_right_pct = 0;
            return;
        }

        base_speed = RT_LOST_SEARCH_SPD;
        if ((rt_lost_recovery / 30) % 2 == 0)
        {
            diff = (rt_last_error >= 0.0f) ? RT_LOST_STEER : -RT_LOST_STEER;
        }
        else
        {
            diff = (rt_last_error >= 0.0f) ? -RT_LOST_STEER : RT_LOST_STEER;
        }
    }
    else
    {
        int target_base = rt_select_base_speed(error, is_cross, is_right_angle);

        if (rt_is_concave_section && error < -0.5f)
        {
            rt_current_speed += RT_SMOOTH_RATE_CONCAVE * ((float)target_base - rt_current_speed);
            base_speed = (int)(rt_current_speed + 0.5f);

            float concave_kp = RT_KP_CONCAVE_LEFT;
            if (error < -1.2f) concave_kp *= 1.2f;

            diff = concave_kp * error + RT_KD_CONCAVE * (error - rt_last_error);

            if (diff < -RT_MAX_STEER_CONCAVE) diff = -RT_MAX_STEER_CONCAVE;
            if (diff > 10) diff = 10;
        }
        else if (rt_is_right_angle)
        {
            rt_current_speed += RT_SMOOTH_RATE_RIGHT_ANGLE * ((float)target_base - rt_current_speed);
            base_speed = (int)(rt_current_speed + 0.5f);

            diff = RT_KP_RIGHT_ANGLE * error + RT_KD_RIGHT_ANGLE * (error - rt_last_error);

            if (diff > RT_MAX_STEER_RIGHT_ANGLE) diff = RT_MAX_STEER_RIGHT_ANGLE;
            else if (diff < -RT_MAX_STEER_RIGHT_ANGLE) diff = -RT_MAX_STEER_RIGHT_ANGLE;
        }
        else
        {
            base_speed = rt_smooth_speed(target_base);

            float kp = rt_get_adaptive_kp(error);

            if (error < -0.3f)
            {
                kp *= RT_LEFT_TURN_BOOST;
            }

            diff = kp * error + RT_KD * (error - rt_last_error);

            float exit_brake = rt_calc_exit_brake(error, rt_last_error, rt_last_last_error);
            diff += exit_brake;

            if (diff > RT_MAX_STEER) diff = RT_MAX_STEER;
            else if (diff < -RT_MAX_STEER) diff = -RT_MAX_STEER;
        }

        rt_last_last_error = rt_last_error;
        rt_last_error = error;
    }

    rt_left_pct  = base_speed + (int)diff;
    rt_right_pct = base_speed - (int)diff;

    if (rt_left_pct > RT_BASE_SPEED_MAX) rt_left_pct = RT_BASE_SPEED_MAX;
    if (rt_left_pct < -RT_BASE_SPEED_MAX) rt_left_pct = -RT_BASE_SPEED_MAX;
    if (rt_right_pct > RT_BASE_SPEED_MAX) rt_right_pct = RT_BASE_SPEED_MAX;
    if (rt_right_pct < -RT_BASE_SPEED_MAX) rt_right_pct = -RT_BASE_SPEED_MAX;
}

void race_track_get_wheel_targets(int *left_pct, int *right_pct)
{
    if (left_pct) *left_pct = rt_left_pct;
    if (right_pct) *right_pct = rt_right_pct;
}

float race_track_get_error(void)
{
    return rt_error;
}

u8 race_track_is_lost(void)
{
    return rt_is_lost;
}

u8 race_track_is_all_black(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (!rt_black[i]) return 0;
    }
    return 1;
}

u8 race_track_is_lost_timeout(void)
{
    return rt_is_lost && (rt_lost_recovery >= RT_LOST_TIMEOUT);
}
