#ifndef __OBSERVER_AS5600_H__
#define __OBSERVER_AS5600_H__

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
    } Observer_AS5600;

    void observer_as5600_init(Observer_AS5600 *self, uint8_t ch);
    void observer_as5600_align(Observer_AS5600 *self, Motor *motor);
    void observer_as5600_update(Observer_AS5600 *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif