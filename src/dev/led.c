#include "led.h"

#define LED_APB RCC_AHBPeriph_GPIOB
#define LED_PORT GPIOB
#define LED_PIN GPIO_Pin_0

void led_init()
{
    RCC_AHBPeriphClockCmd(LED_APB, ENABLE); // 启用APB_GPIOC外设时钟
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = LED_PIN,              //
        .GPIO_Mode = GPIO_Mode_OUT,       // 输出模式
        .GPIO_Speed = GPIO_Speed_Level_3, // 50MHz
        .GPIO_OType = GPIO_OType_PP,      // 推挽输出
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    GPIO_Init(LED_PORT, &GPIO_InitStruct); // 初始化
    led_off();
}
uint8_t led_state = 0;

uint8_t led_get_state()
{
    return led_state;
}

void led_turn()
{
    led_state = 1;
    GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
}

void led_off()
{
    led_state = 0;
    GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
}

void led_toggle()
{
    if (led_state)
        led_off();
    else
        led_turn();
}

#include "delay.h"
void led_test()
{
    led_init();
    while (1)
    {
        led_turn();
        delay_ms(100);
        led_off();
        delay_ms(100);
    }
}