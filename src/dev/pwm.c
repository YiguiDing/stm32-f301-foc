
#include "pwm.h"
#include <stddef.h>

float freq = 1e3;
float Ts = 0.001;
void (*pwm1_on_update)(float Ts) = NULL;

/**
 * @param freq ∈ [1,72k]Hz
 * @param callback 更新回调函数
 */
void pwm1_init()
{
    TIM_Cmd(TIM1, DISABLE); // 关闭定时器
    freq = 1e3;
    Ts = 1 / freq;
    pwm1_on_update = NULL;
    /* 配置GPIO */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // 开启GPIO的时钟
    GPIO_InitTypeDef GPIO_InitStruct = {
        // PA8 -> TIM1_CH1
        // PA9 -> TIM1_CH2
        // PA10 -> TIM1_CH3
        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10, // 引脚
        .GPIO_Mode = GPIO_Mode_AF,                         // 复用
        .GPIO_Speed = GPIO_Speed_Level_3,                  // 高速
        .GPIO_OType = GPIO_OType_PP,                       // 推挽
        .GPIO_PuPd = GPIO_PuPd_NOPULL,                     // 下拉
    };
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*复用功能配置*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);  // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA8] = TIM1_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);  // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA9] = TIM1_CH2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6); // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA10] = TIM1_CH3

    // 配置NVIC
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC优先级 TODO: 优先级分组配置不应当写在此处
    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = TIM1_UP_TIM16_IRQn,  // 中断通道: 定时器1
        .NVIC_IRQChannelPreemptionPriority = 0, // 抢占优先级,0最高优先级
        .NVIC_IRQChannelSubPriority = 0,        // 响应优先级,0最高优先级
        .NVIC_IRQChannelCmd = ENABLE,           // 启用通道
    };
    NVIC_Init(&NVIC_InitStruct); // 初始化NVIC

    /* 配置时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // 开启时钟
    TIM_InternalClockConfig(TIM1);                       // 为时基单元选择内部时钟源

    // 时钟源      输出频率     分辨率      预分频器
    // 72Mhz ->   / 1.098hz     /1000       = 65536
    // 72Mhz ->   / 1khz        /1000       = 72
    // 72Mhz ->   / 2khz        /1000       = 36
    // 72Mhz ->   / 4khz        /1000       = 18
    // 72Mhz ->   / 8khz        /1000       = 9
    // 72Mhz ->   / 14.4khz     /1000       = 5
    // 72Mhz ->   / 18khz       /1000       = 4
    // 72Mhz ->   / 24khz       /1000       = 3
    // 72Mhz ->   / 36khz       /1000       = 2
    // 72Mhz ->   / 72khz       /1000       = 1
    // 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        .TIM_ClockDivision = TIM_CKD_DIV1, // 外部时钟信号滤波器,1分频,不滤波
        // TIM_CounterMode_Up (向上计数) 计数器从 0 开始递增，直到自动重装载值 (ARR)，然后归零并产生计数器溢出事件。
        // TIM_CounterMode_Down(向下计数) 计数器从 0 计数器从 ARR 开始递减，直到 0，然后重新回到 ARR 并产生计数器下溢事件。
        // TIM_CounterMode_CenterAligned1 (中央对齐模式1) 计数器从 0 递增到 ARR-1，产生计数器上溢事件，然后递减到 1，产生计数器下溢事件。 但仅在向下计数阶段（到达 0 之前）产生更新中断/标志 (UIF)
        // TIM_CounterMode_CenterAligned2 (中央对齐模式2)  仅在向上计数阶段（到达 ARR 之前）产生更新中断/标志 (UIF)
        // TIM_CounterMode_CenterAligned3 (中央对齐模式3) 在向上和向下计数阶段均产生更新中断/标志 (UIF)
        .TIM_CounterMode = TIM_CounterMode_CenterAligned2,    // 计数模式
        .TIM_Prescaler = (SystemCoreClock / 1000 / freq) - 1, // 预分频器 PSC = 72Mhz/1000/freq
        .TIM_Period = 1000 - 1,                               // 自动重装器 ARR
        .TIM_RepetitionCounter = 0,                           // 重复计数器，高级定时器才有
    };
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); // 初始化

    // 配置输出比较单元
    TIM_OCInitTypeDef TIM_OCInitStruct = {
        // PWM1在 CNT<CCR 时输出 有效电平，
        // PWM2在 CNT<CCR 时输出 无效电平
        .TIM_OCMode = TIM_OCMode_PWM2,                // 设置输出比较模式
        .TIM_OutputState = TIM_OutputState_Enable,    // 使能输出
        .TIM_OutputNState = TIM_OutputNState_Disable, // 禁用互补输出
        .TIM_Pulse = 0x00000000,                      // 设置CCR捕获比较寄存器的值 [0x0000 and 0xFFFF]
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OCNPolarity = TIM_OCPolarity_High,
        .TIM_OCIdleState = TIM_OCIdleState_Reset,
        .TIM_OCNIdleState = TIM_OCNIdleState_Reset,
    };
    TIM_OC1Init(TIM1, &TIM_OCInitStruct); // 初始化通道1
    TIM_OC2Init(TIM1, &TIM_OCInitStruct); // 初始化通道2
    TIM_OC3Init(TIM1, &TIM_OCInitStruct); // 初始化通道3

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);      // 上面的时基单元初始化函数最后生成了更新事件，这里清除更新事件否则中断函数会立即执行
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // 启用更新中断

    TIM_CtrlPWMOutputs(TIM1, ENABLE); // TIM1作为高级定时器，需通过TIM_CtrlPWMOutputs()函数使能主输出（MOE），否则所有通道输出将被禁止。

    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); // for adc triger,事件映射: 更新事件 => TRGO事件

    TIM_Cmd(TIM1, DISABLE); // 关闭定时器
}

/**
 * 重写中断函数
 */
void TIM1_UP_TIM16_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (pwm1_on_update)
        {
            pwm1_on_update(Ts);
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除更新中断
    }
}
void pwm1_set_freq(float freq)
{
    TIM1->PSC = (SystemCoreClock / 1000 / freq) - 1;
}

void pwm1_set_callback(void (*on_update)(float Ts))
{
    pwm1_on_update = on_update;
}

/**
 * 设置占空比
 * @param duty [0.0f,1.0f]
 */
void pwm1_set_duty(float dutyA, float dutyB, float dutyC)
{
    TIM_SetCompare1(TIM1, dutyA * 1000);
    TIM_SetCompare2(TIM1, dutyB * 1000);
    TIM_SetCompare3(TIM1, dutyC * 1000);
}

void pwm1_start()
{
    TIM_Cmd(TIM1, ENABLE);
}
void pwm1_stop()
{
    TIM_Cmd(TIM1, DISABLE);
}

#include "delay.h"
void pwm_test()
{
    pwm1_init(NULL, 18e3);
    uint16_t cnt = 0;
    while (1)
    {
        pwm1_set_duty(cnt, cnt, cnt); // 示波器测量波形占空比和频率
        cnt++;
        cnt %= 1000;
        delay(100);
    }
}