#include "alarm_control.h"
#include "../../BSP/actuator/buzzer.h"
#include "../../BSP/actuator/led.h"
#include "../app.h"

/* ===================== 可调参数 ===================== */
#define A_BEEP_FREQ             1200
#define A_BEEP_MS               150

#define START_BEEP_FREQ         1000
#define START_BEEP_MS           100

#define FINISHED_BEEP_FREQ      1500
#define FINISHED_BEEP_MS        800

/* alarm_control_update() 由 fsm_update() 按 APP_FSM_PERIOD_MS 调用 */
#define BRAKE_BEEP_PERIOD       (1000 / APP_FSM_PERIOD_MS)   /* 1s */
#define BRAKE_LED_PERIOD        (500 / APP_FSM_PERIOD_MS)    /* 0.5s */

/* ===================== 静态变量 ===================== */
static alarm_event_t s_event = ALARM_EVENT_IDLE;
static u16 s_tick = 0;

static led_mode_t s_stat_mode = LED_MODE_OFF;
static u16 s_stat_tick = 0;

static void update_stat_led(void);

/* ===================== 接口实现 ===================== */

/* 初始化报警模块 */
void alarm_control_init(void)
{
    s_event = ALARM_EVENT_IDLE;
    s_tick = 0;
    s_stat_mode = LED_MODE_OFF;
    s_stat_tick = 0;
    buzzer_off();
    led_alarm_off();
    led_stat_off();
}

/* 设置 LED_STAT 主显示模式 */
void alarm_control_set_stat_mode(led_mode_t mode)
{
    if (mode >= LED_MODE_MAX)
    {
        mode = LED_MODE_OFF;
    }
    s_stat_mode = mode;
    s_stat_tick = 0;
}

void alarm_control_event(alarm_event_t event)
{
    s_event = event;
    s_tick = 0;

    switch (event)
    {
        case ALARM_EVENT_START:
            buzzer_beep(START_BEEP_FREQ, START_BEEP_MS);
            break;

        case ALARM_EVENT_A_POINT:
            buzzer_beep(A_BEEP_FREQ, A_BEEP_MS);
            /* LED_STAT 由模式管理器统一控制，此处不再单独 toggle */
            break;

        case ALARM_EVENT_AVOIDING:
            /* 持续报警由 alarm_control_update() 驱动，此处仅设置事件 */
            buzzer_on(2000);
            led_alarm_on();
            break;

        case ALARM_EVENT_BRAKING:
            /* 周期性提示由 alarm_control_update() 驱动 */
            break;

        case ALARM_EVENT_FINISHED:
            buzzer_beep(FINISHED_BEEP_FREQ, FINISHED_BEEP_MS);
            led_alarm_on();
            break;

        case ALARM_EVENT_IDLE:
        default:
            buzzer_off();
            led_alarm_off();
            break;
    }
}

void alarm_control_update(void)
{
    switch (s_event)
    {
        case ALARM_EVENT_AVOIDING:
            /* 持续声光报警：LED闪烁 + 蜂鸣器常响 */
            if ((s_tick % 25) == 0)  /* 每500ms切换一次(25×20ms) */
            {
                led_alarm_toggle();
            }
            s_tick++;
            buzzer_on(2000);
            break;

        case ALARM_EVENT_BRAKING:
            /* A 点停车：每秒短鸣 + 每 0.5s 闪灯 */
            if ((s_tick % BRAKE_BEEP_PERIOD) == 0)
            {
                buzzer_beep(A_BEEP_FREQ, A_BEEP_MS);
            }
            if ((s_tick % BRAKE_LED_PERIOD) == 0)
            {
                led_alarm_toggle();
            }
            s_tick++;
            break;

        case ALARM_EVENT_IDLE:
        case ALARM_EVENT_START:
        case ALARM_EVENT_A_POINT:
        case ALARM_EVENT_FINISHED:
        default:
            /* 这些事件为一次性动作，已在 event() 中处理 */
            break;
    }

    update_stat_led();
}

/* 根据当前模式刷新 LED_STAT，由 alarm_control_update() 每 20ms 调用一次 */
static void update_stat_led(void)
{
    switch (s_stat_mode)
    {
        case LED_MODE_STEADY_ON:
            led_stat_on();
            break;

        case LED_MODE_SLOW_BLINK:
            /* 周期 2s：亮 1s，灭 1s */
            if ((s_stat_tick % 100) < 50)
            {
                led_stat_on();
            }
            else
            {
                led_stat_off();
            }
            break;

        case LED_MODE_FAST_BLINK:
            /* 周期约 240ms：亮 120ms，灭 120ms */
            if ((s_stat_tick % 12) < 6)
            {
                led_stat_on();
            }
            else
            {
                led_stat_off();
            }
            break;

        case LED_MODE_DOUBLE_FLASH:
        {
            /* 闪 100ms - 灭 100ms - 闪 100ms - 长灭 900ms */
            u16 t = s_stat_tick % 65;
            if (t < 5 || (t >= 10 && t < 15))
            {
                led_stat_on();
            }
            else
            {
                led_stat_off();
            }
            break;
        }

        case LED_MODE_BREATH:
            /* 当前无硬件 PWM，使用慢闪作为呼吸灯占位效果 */
            if ((s_stat_tick % 100) < 50)
            {
                led_stat_on();
            }
            else
            {
                led_stat_off();
            }
            break;

        case LED_MODE_OFF:
        default:
            led_stat_off();
            break;
    }

    s_stat_tick++;
}
