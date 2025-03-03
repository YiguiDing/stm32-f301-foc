#ifndef __LED_H__
#define __LED_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void led_init();
    uint8_t led_get_state();
    void led_turn();
    void led_off();
    void led_toggle();
    void led_test();

#ifdef __cplusplus
}
#endif

#endif