#include "adc.h"
#include "pwm.h"

// 12bit分辨率 0xfff [0,4095]
uint16_t adc_value[16] = {0};
void adc_init()
{
    /* GPIO配置 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // 开启GPIO时钟
    GPIO_InitTypeDef GPIO_InitStruct = {
        // ADC1_IN1 -> PA0
        // ADC1_IN2 -> PA1
        // ADC1_IN3 -> PA2
        // ADC1_IN4 -> PA3
        // ADC1_IN5 -> PA4
        .GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2, // pin_0 pin_1 pin_2
        .GPIO_Mode = GPIO_Mode_AN,                        // 模拟输入输出
        .GPIO_Speed = GPIO_Speed_Level_3,                 // 高速
        .GPIO_OType = GPIO_OType_OD,                      // 开漏输出模式
        .GPIO_PuPd = GPIO_PuPd_NOPULL,                    // 无上拉
    };
    GPIO_Init(GPIOA, &GPIO_InitStruct); // 初始化GPIO

    /* 配置NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC优先级 TODO: 优先级分组配置不应当写在此处
    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = DMA1_Channel1_IRQn, // 中断通道: DMA1-通道1
        // .NVIC_IRQChannel = ADC1_2_IRQn,         // 中断通道: ADC1_2
        .NVIC_IRQChannelPreemptionPriority = 0, // 抢占优先级,0最高优先级
        .NVIC_IRQChannelSubPriority = 0,        // 响应优先级,0最高优先级
        .NVIC_IRQChannelCmd = ENABLE,           // 启用通道
    };
    NVIC_Init(&NVIC_InitStruct); // 初始化NVIC
    /* DMA配置 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 使能DMA时钟
    DMA_InitTypeDef DMA_InitStruct =                   // 配置DMA
        {
            .DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR),           // 外设起始地址
            .DMA_MemoryBaseAddr = (uint32_t)adc_value,                 // 内存起始地址
            .DMA_DIR = DMA_DIR_PeripheralSRC,                          // 方向： 外设 => 内存
            .DMA_BufferSize = 3,                                       // 数据长度3
            .DMA_PeripheralInc = DMA_PeripheralInc_Disable,            // 外设地址不自增
            .DMA_MemoryInc = DMA_MemoryInc_Enable,                     // 内存地址自增
            .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord, // 外设数据大小 半字(uint16_t)
            .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,         // 内存数据大小 半字(uint16_t)
            .DMA_Mode = DMA_Mode_Circular,                             // 循环模式
            .DMA_Priority = DMA_Priority_High,                         // 优先级
            .DMA_M2M = DMA_M2M_Disable,                                // 存储器to存储器(软件触发)
        };
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);       // 初始化DMA
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // 使能DMA中断,传输完成标志位
    DMA_Cmd(DMA1_Channel1, ENABLE);                 // 启用DMA

    /* ADC配置 */
    // 时钟源       预分频          采样周期         采样通道       采样频率
    // 76M            /1              /1.5            /3            16.88M
    // 76M            /6              /1.5            /3            2.81M
    // 76M            /16             /1.5            /3            1.055M
    // 76M            /64             /1.5            /3            263.888k
    // 76M            /128            /1.5            /3            131.944k
    // 76M            /128            /4.5            /3            43.981k
    // 76M            /128            /7.5            /3            26.388k
    // 76M            /128            /19.5           /3            10.149k
    // 76M            /128            /181.5          /3            1.090k
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6);                                    // 配置ADC时钟 最高12M，72/6=12，所以6分频
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);                        // 开启ADC时钟
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_4Cycles5); // 通道1放入序列1，采样时间为1.5个时钟周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_4Cycles5); // 通道2放入序列2，采样时间为1.5个时钟周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_4Cycles5); // 通道3放入序列3，采样时间为1.5个时钟周期

    ADC_InitTypeDef ADC_InitStruct = // 配置ADC
        {
            .ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable,          // 关闭连续转换模式
            .ADC_Resolution = ADC_Resolution_12b,                              // 12bit分辨率
            .ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_9,          // 外部触发源
            .ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge, // 外部触发边沿
            .ADC_DataAlign = ADC_DataAlign_Right,                              // 右对齐
            .ADC_OverrunMode = ADC_OverrunMode_Disable,                        // 禁用 overrun 模式
            .ADC_AutoInjMode = ADC_AutoInjec_Disable,                          // 禁用自动注入模式
            .ADC_NbrOfRegChannel = 3,                                          // 通道数
        };
    ADC_Init(ADC1, &ADC_InitStruct); // 初始化ADC

    // 校准
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single); // 选择校准模式
    ADC_StartCalibration(ADC1);                                  // 开始校准
    while (ADC_GetCalibrationStatus(ADC1))                       // 等待校准完毕
        ;
    // 启动转换
    ADC_Cmd(ADC1, ENABLE);                         // 使能ADC
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY)) // 等待启动
        ;
    // DMA请求配置
    ADC_DMAConfig(ADC1, ADC_DMAMode_Circular); // 循环模式
    ADC_DMACmd(ADC1, ENABLE);                  // 使能ADC_DMA请求 ADC1_DMA_REQUEST -> DMA1_Channel1

    // 中断配置
    // ADC_ITConfig(ADC1, ADC_IT_EOS, ENABLE); // 序列转换完成中断

    // 开始转换(软件触发 或 外部触发)
    ADC_StartConversion(ADC1); //
}

#include "serial.h"
void __attribute__((weak)) adc_on_update(float dt)
{
    // serial_printf(
    //     "adc1: %.6f,%.6f,%.6f\n",
    //     adc_get_value(0) * 3.3f,
    //     adc_get_value(1) * 3.3f,
    //     adc_get_value(2) * 3.3f);
}

void DMA1_Channel1_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC1)) // DMA搬运完成
    {
        adc_on_update(1 / pwm_freq);
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

// void ADC1_2_IRQHandler()
// {
//     if (ADC_GetITStatus(ADC1, ADC_IT_EOS)) // ADC转换完成
//     {
//         adc_on_update(1 / pwm_freq);
//         ADC_ClearITPendingBit(ADC1, ADC_IT_EOS);
//     }
// }

/**
 * @return [0,1] 表示 [0,3.3v]
 */
float adc_get_value(uint8_t ch)
{
    return adc_value[ch] / (float)0xfff; // 读取转换结果
}

#include "serial.h"
#include "delay.h"
#include "pwm.h"
void adc_test()
{
    serial_init();
    pwm_init();
    adc_init();
    while (1)
        ;
}