#ifndef __OBSERVER_I240A2_H__
#define __OBSERVER_I240A2_H__

#include <stdint.h>
#include "foc_math.h"
#include "pid_controller.h"
#include "pll.h"

typedef struct Motor Motor;

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint8_t ch;
        uint8_t addr;
        float offset;
        float theta_mes, theta_hat, omega_hat;
        PLL pll;
    } Observer_I240A2;

    void observer_i240a2_init(Observer_I240A2 *self, uint8_t ch);
    void observer_i240a2_align(Observer_I240A2 *self, Motor *motor);
    void observer_i240a2_update(Observer_I240A2 *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif