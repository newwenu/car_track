#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f10x.h"

/*===========================================================================
 * 板级引脚配置 (Board Support Package)
 *
 * 命名规范: BSP_<模块>_<信号>_<属性>
 *   模块:      MOTOR, TRACE, ULTRASONIC, ENCODER, BUZZER, OLED, KEY, LED, MIC
 *   信号:      功能信号名 (PWM_L, IN1, AO1, ECHO, SCL, SDA, START, EXT 等)
 *   属性:      PORT, PIN, BIT, CLK, TIM, CH, PERIOD, PSC 等
 *   聚合宏:    _ALL 后缀 (如 BSP_TRACE_CLK_ALL = 所有循迹引脚时钟的 OR)
 *
 * 原则: 宏名 == 功能描述, 不绑定物理引脚号。
 *       换引脚只改值, 不改名。所有源文件零改动。
 *===========================================================================*/

/*---------------------------------------------------------------------------
 * 1. 电机 PWM（TIM2）— PA0, PA1
 *---------------------------------------------------------------------------*/
#define BSP_MOTOR_PWM_PORT      GPIOA
#define BSP_MOTOR_PWM_CLK       RCC_APB2Periph_GPIOA
#define BSP_MOTOR_PWM_L_PIN     GPIO_Pin_0      /* 左轮调速 */
#define BSP_MOTOR_PWM_R_PIN     GPIO_Pin_1      /* 右轮调速 */
#define BSP_MOTOR_PWM_TIM       TIM2
#define BSP_MOTOR_PWM_TIM_CLK   RCC_APB1Periph_TIM2
#define BSP_MOTOR_PWM_PERIOD    7199            /* 72MHz/(7199+1)=10kHz */
#define BSP_MOTOR_PWM_PSC       0

/*---------------------------------------------------------------------------
 * 2. 电机方向控制 — PB12~PB15（右侧排针连续4pin）
 *---------------------------------------------------------------------------*/
#define BSP_MOTOR_IN1_PORT      GPIOB
#define BSP_MOTOR_IN1_CLK       RCC_APB2Periph_GPIOB
#define BSP_MOTOR_IN1_PIN       GPIO_Pin_12     /* 左轮方向 IN1 */
#define BSP_MOTOR_IN1_BIT       12

#define BSP_MOTOR_IN2_PORT      GPIOB
#define BSP_MOTOR_IN2_CLK       RCC_APB2Periph_GPIOB
#define BSP_MOTOR_IN2_PIN       GPIO_Pin_13     /* 左轮方向 IN2 */
#define BSP_MOTOR_IN2_BIT       13

#define BSP_MOTOR_IN3_PORT      GPIOB
#define BSP_MOTOR_IN3_CLK       RCC_APB2Periph_GPIOB
#define BSP_MOTOR_IN3_PIN       GPIO_Pin_14     /* 右轮方向 IN3 */
#define BSP_MOTOR_IN3_BIT       14

#define BSP_MOTOR_IN4_PORT      GPIOB
#define BSP_MOTOR_IN4_CLK       RCC_APB2Periph_GPIOB
#define BSP_MOTOR_IN4_PIN       GPIO_Pin_15     /* 右轮方向 IN4 */
#define BSP_MOTOR_IN4_BIT       15

/* 聚合：所有方向引脚时钟 */
#define BSP_MOTOR_DIR_CLK_ALL   (BSP_MOTOR_IN1_CLK)

/*---------------------------------------------------------------------------
 * 3. 编码器 — PB8, PB9（GPIO + EXTI 软件脉冲计数）
 *---------------------------------------------------------------------------*/
#define BSP_ENC_L_PORT          GPIOB
#define BSP_ENC_L_PIN           GPIO_Pin_8      /* 左轮编码器脉冲 */
#define BSP_ENC_L_EXTI          EXTI_Line8
#define BSP_ENC_L_IRQ           EXTI9_5_IRQn

#define BSP_ENC_R_PORT          GPIOB
#define BSP_ENC_R_PIN           GPIO_Pin_9      /* 右轮编码器脉冲 */
#define BSP_ENC_R_EXTI          EXTI_Line9
#define BSP_ENC_R_IRQ           EXTI9_5_IRQn

#define BSP_ENC_CLK_ALL         RCC_APB2Periph_GPIOB

/*---------------------------------------------------------------------------
 * 4. 循迹传感器（5路 ADC 模拟量）— PA2, PA3, PA6, PA7, PB0
 *---------------------------------------------------------------------------*/
#define BSP_TRACE_ADC           ADC1
#define BSP_TRACE_ADC_CLK       RCC_APB2Periph_ADC1
#define BSP_TRACE_CH_COUNT      5

/* AO1 = ADC1_CH2, PA2, 最右传感器 */
#define BSP_TRACE_AO1_PORT      GPIOA
#define BSP_TRACE_AO1_CLK       RCC_APB2Periph_GPIOA
#define BSP_TRACE_AO1_PIN       GPIO_Pin_2
#define BSP_TRACE_AO1_CH        ADC_Channel_2

/* AO2 = ADC1_CH3, PA3, 右传感器 */
#define BSP_TRACE_AO2_PORT      GPIOA
#define BSP_TRACE_AO2_CLK       RCC_APB2Periph_GPIOA
#define BSP_TRACE_AO2_PIN       GPIO_Pin_3
#define BSP_TRACE_AO2_CH        ADC_Channel_3

/* AO3 = ADC1_CH6, PA6, 中间传感器 */
#define BSP_TRACE_AO3_PORT      GPIOA
#define BSP_TRACE_AO3_CLK       RCC_APB2Periph_GPIOA
#define BSP_TRACE_AO3_PIN       GPIO_Pin_6
#define BSP_TRACE_AO3_CH        ADC_Channel_6

/* AO4 = ADC1_CH7, PA7, 左传感器 */
#define BSP_TRACE_AO4_PORT      GPIOA
#define BSP_TRACE_AO4_CLK       RCC_APB2Periph_GPIOA
#define BSP_TRACE_AO4_PIN       GPIO_Pin_7
#define BSP_TRACE_AO4_CH        ADC_Channel_7

/* AO5 = ADC1_CH8, PB0, 最左传感器 */
#define BSP_TRACE_AO5_PORT      GPIOB
#define BSP_TRACE_AO5_CLK       RCC_APB2Periph_GPIOB
#define BSP_TRACE_AO5_PIN       GPIO_Pin_0
#define BSP_TRACE_AO5_CH        ADC_Channel_8

/* 聚合：所有循迹 GPIO 时钟 */
#define BSP_TRACE_CLK_ALL       (BSP_TRACE_AO1_CLK | BSP_TRACE_AO2_CLK | \
                                 BSP_TRACE_AO3_CLK | BSP_TRACE_AO4_CLK | \
                                 BSP_TRACE_AO5_CLK)

/*---------------------------------------------------------------------------
 * 5. 超声波 — PA8(40kHz发射) + PB6(回波捕获)
 *---------------------------------------------------------------------------*/
#define BSP_US_TX_PORT          GPIOA
#define BSP_US_TX_CLK           RCC_APB2Periph_GPIOA
#define BSP_US_TX_PIN           GPIO_Pin_8      /* 40kHz 载波发射 */
#define BSP_US_TX_TIM           TIM1
#define BSP_US_TX_TIM_CLK       RCC_APB2Periph_TIM1

#define BSP_US_ECHO_PORT        GPIOB
#define BSP_US_ECHO_CLK         RCC_APB2Periph_GPIOB
#define BSP_US_ECHO_PIN         GPIO_Pin_6      /* 回波输入捕获 */
#define BSP_US_ECHO_BIT         6
#define BSP_US_ECHO_TIM         TIM4
#define BSP_US_ECHO_TIM_CLK     RCC_APB1Periph_TIM4
#define BSP_US_ECHO_CH          TIM_Channel_1

/*---------------------------------------------------------------------------
 * 6. 蜂鸣器 — PB1（TIM3_CH4 PWM）
 *---------------------------------------------------------------------------*/
#define BSP_BUZZER_PORT         GPIOB
#define BSP_BUZZER_CLK          RCC_APB2Periph_GPIOB
#define BSP_BUZZER_PIN          GPIO_Pin_1
#define BSP_BUZZER_TIM          TIM3
#define BSP_BUZZER_TIM_CLK      RCC_APB1Periph_TIM3

/*---------------------------------------------------------------------------
 * 7. OLED 软件 I2C — PB4(SCL), PB5(SDA)
 *    注意：PA15(JTDI)/PB3(JTDO)/PB4(JNTRST) 是 JTAG 引脚，必须 JTAG_RELEASE()
 *---------------------------------------------------------------------------*/
#define BSP_OLED_SCL_PORT       GPIOB
#define BSP_OLED_SCL_PIN        GPIO_Pin_4      /* 软件 I2C 时钟 */
#define BSP_OLED_SDA_PORT       GPIOB
#define BSP_OLED_SDA_PIN        GPIO_Pin_5      /* 软件 I2C 数据 */
#define BSP_OLED_DC_PORT        GPIOA
#define BSP_OLED_DC_PIN         GPIO_Pin_15     /* 数据/命令 */
#define BSP_OLED_CS_PORT        GPIOB
#define BSP_OLED_CS_PIN         GPIO_Pin_3      /* 片选 */
#define BSP_OLED_CLK_ALL        (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#define BSP_OLED_I2C_ADDR       0x78

/*---------------------------------------------------------------------------
 * 8. 按键 — PA5(内置,启动/模式) + PB11(外接扩展)
 *---------------------------------------------------------------------------*/
#define BSP_KEY_START_PORT      GPIOA
#define BSP_KEY_START_CLK       RCC_APB2Periph_GPIOA
#define BSP_KEY_START_PIN       GPIO_Pin_5      /* 板载 USERKEY */
#define BSP_KEY_START_BIT       5

#define BSP_KEY_EXT_PORT        GPIOB
#define BSP_KEY_EXT_CLK         RCC_APB2Periph_GPIOB
#define BSP_KEY_EXT_PIN         GPIO_Pin_11     /* 外接扩展按键 */
#define BSP_KEY_EXT_BIT         11

/*---------------------------------------------------------------------------
 * 9. LED — PA4(板载状态) + PB10(外接报警)
 *---------------------------------------------------------------------------*/
#define BSP_LED_STAT_PORT       GPIOA
#define BSP_LED_STAT_CLK        RCC_APB2Periph_GPIOA
#define BSP_LED_STAT_PIN        GPIO_Pin_4      /* 板载状态 LED */
#define BSP_LED_STAT_BIT        4

#define BSP_LED_ALARM_PORT      GPIOB
#define BSP_LED_ALARM_CLK       RCC_APB2Periph_GPIOB
#define BSP_LED_ALARM_PIN       GPIO_Pin_10     /* 外接报警 LED */
#define BSP_LED_ALARM_BIT       10

/*---------------------------------------------------------------------------
 * 10. 声控 — PB7（GPIO 数字输入，比较器输出）
 *---------------------------------------------------------------------------*/
#define BSP_MIC_PORT            GPIOB
#define BSP_MIC_CLK             RCC_APB2Periph_GPIOB
#define BSP_MIC_PIN             GPIO_Pin_7
#define BSP_MIC_BIT             7

/*---------------------------------------------------------------------------
 * 11. JTAG 释放 — 使用 PA15/PB3/PB4 前必须调用
 *---------------------------------------------------------------------------*/
#define BSP_JTAG_RELEASE() do { \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); \
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); \
} while(0)

/*---------------------------------------------------------------------------
 * 12. 串口调试（USART1）— PA9/PA10, 经 CH340G 连接 PC
 *     注意：PA9/PA10 不在排针上，仅供 USB-UART 调试
 *---------------------------------------------------------------------------*/
#define BSP_USART_PORT          GPIOA
#define BSP_USART_PORT_CLK      RCC_APB2Periph_GPIOA
#define BSP_USART_CLK           RCC_APB2Periph_USART1
#define BSP_USART_TX_PIN        GPIO_Pin_9
#define BSP_USART_RX_PIN        GPIO_Pin_10

#endif
