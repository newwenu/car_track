#include "led.h"

void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_LED_STAT_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_LED_ALARM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = BSP_LED_STAT_PIN;
    GPIO_Init(BSP_LED_STAT_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_LED_ALARM_PIN;
    GPIO_Init(BSP_LED_ALARM_PORT, &GPIO_InitStructure);

    LED_STAT = 0;
    LED_ALARM = 0;
}

void led_stat_on(void)       { LED_STAT = 1; }
void led_stat_off(void)      { LED_STAT = 0; }
void led_stat_toggle(void)   { LED_STAT = !LED_STAT; }
void led_alarm_on(void)      { LED_ALARM = 1; }
void led_alarm_off(void)     { LED_ALARM = 0; }
void led_alarm_toggle(void)  { LED_ALARM = !LED_ALARM; }
