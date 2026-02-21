#ifndef __OBSERVER_HFI_H__
#define __OBSERVER_HFI_H__

#include <stdint.h>
#include "foc_math.h"
#include "pid.h"
#include "pll.h"

typedef struct Motor Motor;

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        int8_t sign;
        float Ud_injeck;
        float Ialpha_prev, Ibeta_prev;
        float theta_mes, theta_hat, omega_hat;
        float X[3];
        PLL pll;
    } Observer_HFI;
    void observer_hfi_init(Observer_HFI *observer);
    void observer_hfi_update(Observer_HFI *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif