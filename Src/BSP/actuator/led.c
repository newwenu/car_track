#include "led.h"

void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_LED_STAT_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_LED_ALARM_CLK, ENABLE);

    /* 板载状态LED(PA4)：推挽输出，直连电源 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = BSP_LED_STAT_PIN;
    GPIO_Init(BSP_LED_STAT_PORT, &GPIO_InitStructure);

    /* 外部报警LED(PB10)：开漏输出，配合5V外部上拉
     * 电路：+5V → [上拉R] → [LED] → PB10
     *  on():  PB10=0(GND)  → 电流流过LED → 点亮 ✅
     *  off(): PB10=1(高阻) → 被上拉拉到5V → 无电流 → 熄灭 ✅ */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;

    GPIO_InitStructure.GPIO_Pin = BSP_LED_ALARM_PIN;
    GPIO_Init(BSP_LED_ALARM_PORT, &GPIO_InitStructure);

    /* 初始状态：全灭 */
    LED_STAT = 0;     /* 板载LED灭 */
    LED_ALARM = 1;    /* 外部LED灭（高阻态，被上拉到5V） */
}

void led_stat_on(void)       { LED_STAT = 1; }
void led_stat_off(void)      { LED_STAT = 0; }
void led_stat_toggle(void)   { LED_STAT = !LED_STAT; }
void led_alarm_on(void)      { LED_ALARM = 1; }
void led_alarm_off(void)     { LED_ALARM = 0; }
void led_alarm_toggle(void)  { LED_ALARM = !LED_ALARM; }
