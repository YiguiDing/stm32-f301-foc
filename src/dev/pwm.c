
#include "pwm.h"
float pwm_freq;
/**
 * 产生PWM波，精度1% 频率1k 占空比默认50% 可调
 */
void pwm_init()
{
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

    // enable pin
    GPIO_InitTypeDef GPIO_InitStruct_EN = {
        .GPIO_Pin = GPIO_Pin_11,          // 13号引脚
        .GPIO_Mode = GPIO_Mode_OUT,       // 输出模式
        .GPIO_Speed = GPIO_Speed_Level_3, // 50MHz
        .GPIO_OType = GPIO_OType_PP,      // 推挽输出
        .GPIO_PuPd = GPIO_PuPd_DOWN,      // 下拉
    };
    GPIO_Init(GPIOA, &GPIO_InitStruct_EN); // 初始化
    pwm_driver_enable(Bit_RESET);

    /*复用功能配置*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);  // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA8] = TIM1_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);  // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA9] = TIM1_CH2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6); // [AF6(SPI2/I2S2/SPI3/I2S3/TIM1/Infrared),PA10] = TIM1_CH3

    // 配置NVIC
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC优先级 TODO: 优先级分组配置不应当写在此处
    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = TIM1_UP_TIM16_IRQn,  // 中断通道: 定时器1
        .NVIC_IRQChannelPreemptionPriority = 0, // 抢占优先级,0最高优先级
        .NVIC_IRQChannelSubPriority = 1,        // 响应优先级,0最高优先级
        .NVIC_IRQChannelCmd = ENABLE,           // 启用通道
    };
    NVIC_Init(&NVIC_InitStruct); // 初始化NVIC

    /* 配置时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // 开启时钟
    TIM_InternalClockConfig(TIM1);                       // 为时基单元选择内部时钟源

    // 时钟源       预分频器      分辨率       输出频率
    // 72Mhz ->     /72     ->   /1000 =     1khz
    // 72Mhz ->     /7.2    ->   /1000 =     10khz
    // 72Mhz ->     /3.6    ->   /1000 =     20khz
    // 72Mhz ->     /5      ->   /1000 =     14.4khz
    // 72Mhz ->     /4      ->   /1000 =     18khz
    // 72Mhz ->     /3      ->   /1000 =     24khz
    // 72Mhz ->     /2      ->   /1000 =     36khz
    // 72Mhz ->     /1      ->   /1000 =     72khz
    // 72Mhz ->     /1      ->   /3000 =     24khz
    // 72Mhz ->     /1.8    ->   /2000 =     20khz
    // 72Mhz ->     /1.8    ->   /2000 =     20khz
    // 72Mhz ->     /2      ->   /4000 =     8khz
    // 72Mhz ->     /0.9    ->   /4000 =     20khz
    // 72Mhz ->     /1      ->   /4000 =     18khz
    // 72Mhz ->     /7200  ->   /10000 =     1hz(1s)
    pwm_freq = 18e3f; // 18khz
    // 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
        .TIM_ClockDivision = TIM_CKD_DIV1,     // 外部时钟信号滤波器,1分频,不滤波
        .TIM_CounterMode = TIM_CounterMode_Up, // 计数模式，向上计数
        .TIM_Prescaler = 4 - 1,                // 预分频器 PSC
        .TIM_Period = 1000 - 1,                // 自动重装器 ARR
        .TIM_RepetitionCounter = 0,            // 重复计数器，高级定时器才有
    };
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); // 初始化

    // 配置输出比较单元
    TIM_OCInitTypeDef TIM_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1,                // 设置输出比较模式
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

    TIM_Cmd(TIM1, ENABLE); // 启动定时器
}

void __attribute__((weak)) pwm_on_update(float dt)
{
}
/**
 * 重写中断函数
 */
void TIM1_UP_TIM16_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        pwm_on_update(1 / pwm_freq);
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除更新中断
    }
}
/**
 * 设置占空比
 * @param duty [0.0f,1.0f]
 */
void pwm_set_duty(float dutyA, float dutyB, float dutyC)
{
    TIM_SetCompare1(TIM1, dutyA * 1000);
    TIM_SetCompare2(TIM1, dutyB * 1000);
    TIM_SetCompare3(TIM1, dutyC * 1000);
}

void pwm_driver_enable(BitAction flag)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, flag);
}

#include "delay.h"
void pwm_test()
{
    pwm_init();
    uint16_t cnt = 0;
    while (1)
    {
        pwm_set_duty(cnt, cnt, cnt); // 示波器测量波形占空比和频率
        cnt++;
        cnt %= 1000;
        delay(100);
    }
}