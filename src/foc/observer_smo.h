#ifndef __OBSERVER_SMO_H__
#define __OBSERVER_SMO_H__

#include <stdint.h>
#include "foc.h"
#include "pid.h"
#include "pll.h"

typedef struct Motor Motor;

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        float Ialpha_hat, Ibeta_hat;
        float Ealpha_hat, Ebeta_hat;
        float Hgain, delta;
        float theta_hat, omega_hat;
        LowpassFilter lpf_omega;
        PLL pll;
    } Observer_SMO;
    void observer_smo_init(Observer_SMO *observer);
    void observer_smo_update(Observer_SMO *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif