#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void adc_init();
    extern uint16_t adc_value[16];
    void adc_on_update(float dt);
    float adc_get_value(uint8_t ch);
    void adc_test();

#ifdef __cplusplus
}
#endif

#endif