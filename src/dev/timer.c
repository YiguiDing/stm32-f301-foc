#include "timer.h"

float timer_freq = 1;
void timer_init()
{
    // TIM2 通用 预分频系数16bit 计数器32位

    // 配置时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启TIM时钟
    TIM_InternalClockConfig(TIM2);                       // 为时基单元选择内部时钟源

    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC优先级 TODO: 优先级分组配置不应当写在此处
    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = TIM2_IRQn,           // 中断通道
        .NVIC_IRQChannelPreemptionPriority = 0, // 抢占优先级,0最高优先级
        .NVIC_IRQChannelSubPriority = 0,        // 响应优先级,0最高优先级
        .NVIC_IRQChannelCmd = ENABLE,           // 启用通道
    };
    NVIC_Init(&NVIC_InitStruct); // 初始化NVIC

    // 配置时基单元
    // 时钟源   预分频器    自动重装器     输出频率
    // 72Mhz    /72         /1000,000   =1hz(1s)
    timer_freq = 1e3;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        .TIM_ClockDivision = TIM_CKD_DIV1,     // 外部时钟信号滤波器,1分频,不滤波
        .TIM_CounterMode = TIM_CounterMode_Up, // 计数模式，向上计数
        .TIM_Prescaler = 72 - 1,               // 预分频器 PSC
        .TIM_Period = 1000000 - 1,             // 自动重装器 ARR
        .TIM_RepetitionCounter = 0,            // 重复计数器(高级定时器独有)
    };
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct); // 初始化时基单元
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);            // 上面的时基单元初始化函数最后生成了更新事件，这里清除更新事件否则中断函数会立即执行
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);       // 启用更新中断
    TIM_Cmd(TIM2, ENABLE);                           // 启动定时器
}

// [0,0xFFFFFFFF] = [0,4294967295]
uint32_t seconds = 0;      // [0,0xFFFFFFFF] -> 4294967295/60/60/24/365  = 136年    溢出一次
uint32_t milliseconds = 0; // [0,0xFFFFFFFF] -> 4294967295/1000/60/60/24 = 49天     溢出一次
uint32_t microseconds = 0; // [0,0xFFFFFFFF] -> 4294967295/1000/1000/60  = 71分钟   溢出一次

/**
 * 重写中断函数
 */
void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        seconds += 1;
        milliseconds += 1000;
        microseconds += 1000000;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除更新中断
    }
}

uint32_t timer_get_us()
{
    return microseconds + TIM2->CNT;
}

uint32_t timer_get_ms()
{
    return milliseconds + TIM2->CNT / 1000;
}

uint32_t timer_get_s()
{
    return seconds;
}

/**
 * 获取本次和上次调用该函数的间隔,单位us
 * @return [1,0xFFFFFFFF]
 */
uint32_t timer_get_dt_us()
{
    /** 全局变量 */
    static uint32_t prev_us = 0;
    static uint32_t dt_us = 1; // 最小为1 防止某些地方计算 x/dt

    uint32_t now_us = timer_get_us();
    // 处理边界情况(溢出)
    dt_us = now_us > prev_us ? now_us - prev_us : dt_us;

    prev_us = now_us;

    return dt_us;
}

#include "serial.h"
#include "delay.h"
void timer_test()
{
    timer_init();
    serial_init();
    while (1)
    {
        serial_printf("%d,%d\r\n", timer_get_us(), timer_get_dt_us());
        delay(100);
    }
}