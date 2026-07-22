#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

/* Pin definitions */
#define BSP_US_TX_PORT     GPIOA
#define BSP_US_TX_PIN      GPIO_Pin_8
#define BSP_US_TX_CLK       RCC_APB2Periph_GPIOA
#define BSP_US_TX_TIM       TIM1
#define BSP_US_TX_TIM_CLK   RCC_APB2Periph_TIM1

#define BSP_US_ECHO_PORT    GPIOB
#define BSP_US_ECHO_PIN     GPIO_Pin_6
#define BSP_US_ECHO_CLK     RCC_APB2Periph_GPIOB
#define BSP_US_ECHO_TIM     TIM4
#define BSP_US_ECHO_TIM_CLK RCC_APB1Periph_TIM4
#define BSP_US_ECHO_CH      TIM_Channel_1

void  ultrasonic_init(void);
void  ultrasonic_start(void);
float ultrasonic_get_distance(void);
float ultrasonic_measure_blocking(void);

void ultrasonic_get_raw(u16 *echo_us, u8 *ok, u8 *listening);
void ultrasonic_set_pull(u8 pull_up);
u8   ultrasonic_get_pull(void);

#endif
