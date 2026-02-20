#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void pwm1_init();
    void pwm1_set_freq(float freq);
    void pwm1_set_callback(void (*on_update)(float Ts));
    void pwm1_set_duty(float dutyA, float dutyB, float dutyC);
    void pwm1_enable();
    void pwm1_disable();
#ifdef __cplusplus
}
#endif

#endif