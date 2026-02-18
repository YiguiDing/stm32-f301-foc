#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ADC_CHANNEL_NUM 3
    // 12bit分辨率 0xfff [0,4095]
    extern uint16_t adc_value[ADC_CHANNEL_NUM];

#define ADC_GET_VALUE(ch) (adc_value[ch] / (float)0xfff)

    void adc_init();
    float adc_get_value(uint8_t ch);
    void adc_set_callback(void (*on_update)());
    void adc_test();

#ifdef __cplusplus
}
#endif

#endif