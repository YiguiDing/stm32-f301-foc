#ifndef __DEV_H__
#define __DEV_H__

#include "stm32f30x.h"

#include "led.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "serial.h"
#include "i2c.h"
#include "delay.h"
#include <FreeRTOS.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void dev_init();
#define dev_red_adc(ch) adc_get_value(ch)
#define dev_set_pwm_duty(a, b, c) pwm_set_duty(a, b, c)
#define dev_set_pwm_enable(flag) pwm_driver_enable(flag)
// #ifdef INC_FREERTOS_H
#if 0
    #define dev_delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS)
    #else
    #define dev_delay(ms) delay_ms(ms)
#endif

#ifdef __cplusplus
}
#endif

#endif