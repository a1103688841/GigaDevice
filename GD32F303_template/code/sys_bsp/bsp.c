/************************************************** 
 * @Author: shuren
 * @Date: 2022-02-18 15:06:48
 * @LastEditTime: 2022-02-18 15:24:59
 * @LastEditors: shuren
 * @Description: 
 * @FilePath: \GD32F303\code\sys_bsp\bsp.c
 * @桃之夭夭，灼灼其华。之子于归， 宜其室家。
 **************************************************/
#include "base_typle.h"
#include "gd32f30x.h"
#include "systick.h"

#define LED_GPIO    GPIOE
#define LED_PIN     GPIO_PIN_3
#define LED_CLOCK   RCU_GPIOE
static uint8_t led_state;
static void led_gpio_init()
{
    rcu_periph_clock_enable(LED_CLOCK);
    gpio_init(LED_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, LED_PIN);    
}
void led_off()
{
    led_state = OFF;
    gpio_bit_reset(LED_GPIO, LED_PIN);
}
void led_on()
{
    led_state = ON;
    gpio_bit_set(LED_GPIO, LED_PIN);
}
void led_toggle()
{
    (led_state == ON)?(led_off()):(led_on());
}
void bsp_init()
{
    systick_config();
		led_gpio_init();
}
