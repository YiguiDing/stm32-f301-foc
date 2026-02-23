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

#define pwm1_set_duty(dutyA, dutyB, dutyC)   \
    {                                        \
        TIM_SetCompare1(TIM1, dutyA * 1000); \
        TIM_SetCompare2(TIM1, dutyB * 1000); \
        TIM_SetCompare3(TIM1, dutyC * 1000); \
    }
#ifndef pwm1_set_duty
    void pwm1_set_duty(float dutyA, float dutyB, float dutyC);
#endif

    void pwm1_enable();
    void pwm1_disable();
#ifdef __cplusplus
}
#endif

#endif