#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        uint32_t prev_us;
        uint32_t dt_us;
    } timer_t;

    void timer_init();
    uint32_t timer_get_us();
    uint32_t timer_get_ms();
    uint32_t timer_get_s();
    uint32_t timer_get_dt_us();
    void timer_test();
#ifdef __cplusplus
}
#endif

#endif