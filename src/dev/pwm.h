#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

    extern float pwm_freq;
    void pwm_init();
    void pwm_on_update(float dt);
    void pwm_set_duty(float dutyA, float dutyB, float dutyC);
    void pwm_driver_enable(BitAction flag);

#ifdef __cplusplus
}
#endif

#endif