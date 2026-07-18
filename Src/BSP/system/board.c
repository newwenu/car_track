#include "board.h"
#include "delay.h"
#include "usart.h"
#include "mic.h"

void all_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    BSP_JTAG_RELEASE();

    delay_init();

    led_init();
    key_init();
    mic_init();

    pwm_motor_init(BSP_MOTOR_PWM_PERIOD, BSP_MOTOR_PWM_PSC);
    motor_init();

    encoder_init();

    trace_init();

    ultrasonic_init();
    ultrasonic_start();

    pwm_buzzer_init(7199, 0);
    buzzer_off();

    uart_init(9600);
    printf("System Ready\r\n");

    oled_spi_init();
    oled_spi_clear();
    oled_spi_show_string(0, 0, (u8*)"System Ready", 16);
}
