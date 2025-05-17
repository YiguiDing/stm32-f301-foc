#include "i2c.h"
#include "stm32f30x.h"

void i2c_init()
{
    /* 配置GPIO */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // 开启GPIO的时钟

    // PB8 -> I2C1_SCL
    GPIO_InitTypeDef GPIO_I2C1_SCL = {
        .GPIO_Pin = GPIO_Pin_8,           // 引脚
        .GPIO_Mode = GPIO_Mode_AF,        // 复用
        .GPIO_Speed = GPIO_Speed_Level_3, // 高速
        .GPIO_OType = GPIO_OType_PP,      // 推挽
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    GPIO_Init(GPIOB, &GPIO_I2C1_SCL);
    // PB9 -> I2C1_SDA
    GPIO_InitTypeDef GPIO_I2C1_SDA = {
        .GPIO_Pin = GPIO_Pin_9,           // 引脚
        .GPIO_Mode = GPIO_Mode_AF,        // 复用
        .GPIO_Speed = GPIO_Speed_Level_3, // 高速
        .GPIO_OType = GPIO_OType_OD,      // 开漏
        .GPIO_PuPd = GPIO_PuPd_UP,        // 上拉
    };
    GPIO_Init(GPIOB, &GPIO_I2C1_SDA);

    /*复用功能配置*/
    // [STM32F301x6 STM32F301x8 DS9895 Rev 8] [page：46/141] [Table 15. Alternate functions for Port B]
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4); // [AF4(I2C1),PB8] = I2C1_SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4); // [AF4(I2C1),PB9] = I2C1_SDA

    /* 配置I2C1 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); // 开启I2C时钟

    // [RM0366 Reference manual] [page：681/874] [Table 90. Timing settings for fI2CCLK of 48 MHz]
    uint8_t PRESC = 0x09 - 1, // Timing prescaler                   // 72M/9 = 8M = 125 ns
        SCLDEL = 0x04 - 1,    // Data setup time                    // 4 x 125 ns = 500 ns
        SDADEL = 0x03,        // Data hold time                     // 3 x 125 ns = 375 ns
        SCLH = 0x04 - 1,      // SCL high period (controller mode)  // 4 x 125 ns = 500 ns
        SCLL = 0x0A - 1;      // SCL low period (controller mode)   // 10 x 125 ns = 1250 ns

    I2C_InitTypeDef I2C_InitStruct = {
        .I2C_Timing = PRESC << 28 | SCLDEL << 20 | SDADEL << 16 | SCLH << 8 | SCLL << 0,
        .I2C_AnalogFilter = I2C_AnalogFilter_Disable,            // 模拟滤波
        .I2C_DigitalFilter = 0x00,                               // 数字滤波
        .I2C_Mode = I2C_Mode_I2C,                                // I2C模式
        .I2C_OwnAddress1 = 0x01,                                 // 本机地址
        .I2C_Ack = I2C_Ack_Enable,                               // 自动回应ACK
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit, // 7bit地址模式
    };
    I2C_Init(I2C1, &I2C_InitStruct); // 初始化
    I2C_Cmd(I2C1, ENABLE);           // 启动
}
#define I2C_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT ((uint32_t)(300 * I2C_FLAG_TIMEOUT))
uint8_t i2c_await(uint32_t flag)
{
    uint32_t timer = I2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, flag) == RESET)
        if ((timer--) == 0)
            return 1; // time_out
    return 0;
}
uint8_t i2c_await_not(uint32_t flag)
{
    uint32_t timer = I2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, flag) != RESET)
        if ((timer--) == 0)
            return 1; // time_out
    return 0;
}
uint8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    // 多字节写入时序
    // | 主 | Start | DEV_ADD+W |     | REG_ADD |     | DATA |     | DATA |     | ...... | DATA |     | Stop |
    // | 从 |       |           | ACK |         | ACK |      | ACK |      | ACK | ...... |      | ACK |      |
    uint8_t is_timeout = 0;
    // #########################################################################################
    /* Test on BUSY Flag */
    if (!(is_timeout = i2c_await(I2C_ISR_BUSY)))
        goto _stop_;
    // #########################################################################################
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, dev_addr, 1 + len, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
    /* Wait until TXIS flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_TXIS)))
        goto _stop_;
    // #########################################################################################
    /* Send Register address */
    I2C_SendData(I2C1, reg_addr);
    /* Wait until TXIS flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_TXIS)))
        goto _stop_;
    // #########################################################################################
    /* send data */
    while (len--)
    {
        /* Write data to TXDR */
        I2C_SendData(I2C1, *(data++));
        /* Wait until TXIS flag is set */
        if ((is_timeout = i2c_await(I2C_ISR_TXIS)))
            goto _stop_;
    }
    // #########################################################################################
    /* Wait until STOPF flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_STOPF)))
        goto _stop_;
_stop_:
    /* Clear STOPF flag */
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
    // #########################################################################################
    return is_timeout;
}
uint8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    // 多字节读取时序
    // | 主 | Start | DEV_ADD+W |     | REG_ADD |     | Start | AD+R |     | DATA |     | DATA |     | ...... | DATA |     | Stop |
    // | 从 |       |           | ACK |         | ACK |       |      | ACK |      | ACK |      | ACK | ...... |      | ACK |      |
    uint8_t is_timeout = 0;
    // #########################################################################################
    /* Test on BUSY Flag */
    if (is_timeout = i2c_await_not(I2C_ISR_BUSY))
    {
        serial_printf("0\n");
        goto _stop_;
    }
    // #########################################################################################
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, dev_addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    /* Wait until TXIS flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_TXIS)))
    {
        serial_printf("1\n");
        goto _stop_;
    }
    // #########################################################################################
    /* Send Register address */
    I2C_SendData(I2C1, reg_addr);
    /* Wait until TC flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_TC)))
    {
        serial_printf("2\n");
        goto _stop_;
    }
    // #########################################################################################
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, dev_addr, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
    // #########################################################################################
    /* reveive data */
    while (len--)
    {
        /* Wait until RXNE flag is set */
        if ((is_timeout = i2c_await(I2C_ISR_RXNE)))
        {
            serial_printf("3\n");
            goto _stop_;
        }
        /* Read data from RXDR */
        *(data++) = I2C_ReceiveData(I2C1);
    }
    // #########################################################################################
    /* Wait until STOPF flag is set */
    if ((is_timeout = i2c_await(I2C_ISR_STOPF)))
    {
        serial_printf("4\n");
        goto _stop_;
    }
_stop_:
    /* Clear STOPF flag */
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
    // #########################################################################################
    return is_timeout;
}

#include "serial.h"
#include "delay.h"

void i2c_test()
{
    serial_init();
    i2c_init();
    uint8_t data[2] = {0};
    uint16_t angle = 0x0000;
    serial_printf("i2c_init\n");
    while (1)
    {
        i2c_read(0x36 << 1, 0x0E, data, 2);
        angle = data[0] << 8 | data[1];
        serial_printf("angle:%d.\n", angle);
        delay(10);
    }
}