#include "serial.h"
#include <stdio.h>
#include <stdarg.h> // for vsnprintf
#include <string.h> // for memcpy

#define MAX_RX_BUFFER_SIZE 255
#define MAX_TX_BUFFER_SIZE 255

// 接收双缓冲区，DMA将收到的数据搬运到缓冲区1，接收完毕后，CPU读取解析处理缓冲区1，同时DMA将后续收到的数据搬运到缓冲区2，彼此交替使用，避免数据覆盖
// 发送双缓冲区，CPU将要发送的数据填充的缓冲区1，填充完毕后，DMA搬运缓冲区1至串口发送，同时CPU将后续要发送的数据填充缓冲区2，彼此交替使用，避免数据覆盖
uint8_t tx_buf[2][MAX_TX_BUFFER_SIZE], rx_buf[2][MAX_RX_BUFFER_SIZE];
int8_t tx_buf_idx = -1, rx_buf_idx = -1;

// 获取下一个接收缓冲区
uint8_t *serial_next_rx_buffer()
{
    return rx_buf[rx_buf_idx = ++rx_buf_idx % 2];
}
// 获取下一个发送缓冲区
uint8_t *serial_next_tx_buffer()
{
    return tx_buf[tx_buf_idx = ++tx_buf_idx % 2];
}

void serial_init()
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 开启USART1的时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);    // 开启GPIOB的时钟

    /*GPIO发送脚配置*/
    GPIO_InitTypeDef GPIO_InitStruct_TX = {
        .GPIO_Pin = GPIO_Pin_6,           // pin_6
        .GPIO_Mode = GPIO_Mode_AF,        // 复用
        .GPIO_Speed = GPIO_Speed_Level_3, // 高速
        .GPIO_OType = GPIO_OType_PP,      // 推挽
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    /*GPIO接收脚配置*/
    GPIO_InitTypeDef GPIO_InitStruct_RX = {
        .GPIO_Pin = GPIO_Pin_7,           // pin_7
        .GPIO_Mode = GPIO_Mode_AF,        // 复用
        .GPIO_Speed = GPIO_Speed_Level_3, // 高速
        .GPIO_OType = GPIO_OType_OD,      // 开漏
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    GPIO_Init(GPIOB, &GPIO_InitStruct_TX);
    GPIO_Init(GPIOB, &GPIO_InitStruct_RX);

    /*复用功能配置*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_7); // [AF7(USART1/USART2/USART3/GPCOMP6),PB6] = USART1_TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_7); // [AF7(USART1/USART2/USART3/GPCOMP6),PB7] = USART1_RX

    /* 配置NVIC */
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC优先级 TODO: 优先级分组配置不应当写在此处
    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannel = USART1_IRQn,         // 中断通道: USART1_IRQn
        .NVIC_IRQChannelPreemptionPriority = 0, // 抢占优先级,0最高优先级
        .NVIC_IRQChannelSubPriority = 0,        // 响应优先级,0最高优先级
        .NVIC_IRQChannelCmd = ENABLE,           // 启用通道
    };
    NVIC_Init(&NVIC_InitStruct); // 初始化NVIC

    /* 配置DMA */
    // ref page:155/874 pdf: https://www.st.com.cn/resource/en/reference_manual/rm0366-stm32f301x68-and-stm32f318x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
    // USART1_TX -> Channel 4
    // USART1_RX -> Channel 5
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 使能DMA时钟
    DMA_InitTypeDef DMA_Init_USART1_TX =               // 配置DMA
        {
            .DMA_PeripheralBaseAddr = (uint32_t)&(USART1->TDR),      // 外设起始地址
            .DMA_MemoryBaseAddr = (uint32_t)serial_next_tx_buffer(), // 内存起始地址
            .DMA_DIR = DMA_DIR_PeripheralDST,                        // 方向： 内存 => 外设
            .DMA_BufferSize = MAX_TX_BUFFER_SIZE,                    // 数据长度
            .DMA_PeripheralInc = DMA_PeripheralInc_Disable,          // 外设地址不自增
            .DMA_MemoryInc = DMA_MemoryInc_Enable,                   // 内存地址自增
            .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,   // 外设数据大小
            .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,           // 内存数据大小
            .DMA_Mode = DMA_Mode_Normal,                             // 普通模式（不循环）
            .DMA_Priority = DMA_Priority_High,                       // 优先级
            .DMA_M2M = DMA_M2M_Disable,                              // 存储器to存储器（软件触发）
        };
    DMA_InitTypeDef DMA_Init_USART1_RX = // 配置DMA
        {
            .DMA_PeripheralBaseAddr = (uint32_t)&(USART1->RDR),      // 外设起始地址
            .DMA_MemoryBaseAddr = (uint32_t)serial_next_rx_buffer(), // 内存起始地址
            .DMA_DIR = DMA_DIR_PeripheralSRC,                        // 方向： 外设  => 内存
            .DMA_BufferSize = MAX_RX_BUFFER_SIZE,                    // 数据长度
            .DMA_PeripheralInc = DMA_PeripheralInc_Disable,          // 外设地址不自增
            .DMA_MemoryInc = DMA_MemoryInc_Enable,                   // 内存地址自增
            .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,   // 外设数据大小
            .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,           // 内存数据大小
            .DMA_Mode = DMA_Mode_Normal,                             // 普通模式（不循环）
            .DMA_Priority = DMA_Priority_High,                       // 优先级
            .DMA_M2M = DMA_M2M_Disable,                              // 存储器to存储器（软件触发）
        };
    DMA_Init(DMA1_Channel4, &DMA_Init_USART1_TX); // 初始化DMA
    DMA_Init(DMA1_Channel5, &DMA_Init_USART1_RX); // 初始化DMA
    DMA_Cmd(DMA1_Channel4, DISABLE);              // 关闭DMA发送搬运（按需开启）
    DMA_Cmd(DMA1_Channel5, ENABLE);               // 启用DMA接收搬运（始终开启）

    /* 配置USART */
    USART_InitTypeDef USART_InitStruct = {
        .USART_BaudRate = 1000000,                                   // 波特率 common:115200 max:100,0000 limit:200,0000
        .USART_WordLength = USART_WordLength_8b,                     // 字长8比特
        .USART_StopBits = USART_StopBits_1,                          // 停止位长度1
        .USART_Parity = USART_Parity_No,                             // 不奇偶校验
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,                 // 收发模式
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None, // 硬件流控制无
    };
    USART_Init(USART1, &USART_InitStruct); // 初始化
    /* USART.NVIC配置 */
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // 使能溢出中断、空闲中断
    /* USART.DMA 配置 */
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE); // 使能DMA收发请求
    /*启动USART*/
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler()
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
    {
        DMA_Cmd(DMA1_Channel5, DISABLE);                                           // 关闭DMA
        uint8_t *data = (uint8_t *)DMA1_Channel5->CMAR;                            // 获取内存起始地址
        uint16_t len = MAX_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5); // 获取数据长度
        DMA1_Channel5->CMAR = (uint32_t)serial_next_rx_buffer();                   // 切换双缓冲区
        DMA_SetCurrDataCounter(DMA1_Channel5, MAX_RX_BUFFER_SIZE);                 // 设置DMA接收最大数据长度
        DMA_ClearFlag(DMA1_FLAG_TE5);                                              // 清除错误标志（否则无法重启DMA） "This EN bit cannot be set again by software to reactivate the channel x, until the TEIFx bit of the DMA_ISR register is set."
        USART_ClearFlag(USART1, USART_FLAG_ORE);                                   // 清除溢出标志（否则DMA搬运异常）
        DMA_Cmd(DMA1_Channel5, ENABLE);                                            // 重启DMA
        if (len > 0)
            serial_on_receive(data, len);               // 处理收到的数据
        USART_ClearITPendingBit(USART1, USART_IT_IDLE); // 清除中断挂起标志
    }
}

void serial_send(uint8_t *data, uint16_t len)
{
    if (len > MAX_TX_BUFFER_SIZE)
        return;
    if (data != tx_buf[0] && data != tx_buf[1]) // 确保地址是双缓冲区的地址
        return;
    // DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
    if (DMA1_Channel4->CCR & DMA_CCR_EN)                  // DMA开启
        while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET) // 阻塞，等待DMA发送完毕
            ;
    DMA_Cmd(DMA1_Channel4, DISABLE);            // 关闭DMA "in noncircular mode","The DMA channel must be disabled to reload a new number of data items into the DMA_CNDTRx register."
    DMA_ClearFlag(DMA1_FLAG_TC4);               // 清除传输完成标志
    DMA1_Channel4->CMAR = (uint32_t)data;       // 内存地址
    DMA_SetCurrDataCounter(DMA1_Channel4, len); // 发送字节数
    DMA_Cmd(DMA1_Channel4, ENABLE);             // 启用
}

__attribute__((weak)) void serial_on_receive(uint8_t *data, uint16_t len)
{
    serial_printf("device have received %d byte:", len);
    for (uint16_t idx = 0; idx < len; idx++)
        serial_printf(" %x", data[idx]);
    serial_printf("\r\n");
}
void serial_printf(const char *format, ...)
{
    uint8_t *data = serial_next_tx_buffer(); // 获取发送缓冲区
    va_list args;
    va_start(args, format);
    int32_t len = vsnprintf((char *)data, MAX_TX_BUFFER_SIZE, format, args);
    va_end(args);
    if (len > 0)
        serial_send(data, len);
}

#include "delay.h"
void serial_test_printf()
{
    serial_init();
    uint8_t idx = 0;
    while (1)
    {
        serial_printf("Hello World!\n");
        serial_printf("cnt:%d.\n", idx++);
        delay(500);
    }
}

void serial_test_send()
{
    serial_init();
    uint8_t idx = 0;
    while (1)
    {
        uint8_t *data = serial_next_tx_buffer();
        uint16_t len = 0;
        data[len++] = 0x55;
        data[len++] = 0xAA;
        data[len++] = 0x01;
        data[len++] = 0x02;
        data[len++] = 0x03;
        data[len++] = idx++;
        serial_send(data, len);
        delay(500);
    }
}