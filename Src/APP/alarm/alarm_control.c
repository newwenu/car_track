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

/* ===================== 接口实现 ===================== */

void alarm_control_init(void)
{
    s_event = ALARM_EVENT_IDLE;
    s_tick = 0;
    buzzer_off();
    led_alarm_off();
    led_stat_off();
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
            led_stat_toggle();
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
            /* 持续声光报警 */
            led_alarm_on();
            buzzer_on(1000);
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
}
