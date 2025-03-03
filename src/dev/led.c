#include "led.h"

void led_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // 启用APB_GPIOC外设时钟
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = GPIO_Pin_13,          // 13号引脚
        .GPIO_Mode = GPIO_Mode_OUT,       // 输出模式
        .GPIO_Speed = GPIO_Speed_Level_3, // 50MHz
        .GPIO_OType = GPIO_OType_PP,      // 推挽输出
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    GPIO_Init(GPIOC, &GPIO_InitStruct); // 初始化
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
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
}

void led_off()
{
    led_state = 0;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
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