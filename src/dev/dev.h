#ifndef __DEV_H__
#define __DEV_H__

#include "stm32f30x.h"

#include "swo.h"
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
#define dev_get_adc_value(ch) adc_get_value(ch)
#define dev_set_pwm1_duty(a, b, c) pwm1_set_duty(a, b, c)
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