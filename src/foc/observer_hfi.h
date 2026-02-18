#ifndef __OBSERVER_HFI_H__
#define __OBSERVER_HFI_H__

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
        float injeck_ud, injeck_uq;
        float injeck_ualpha, injeck_ubeta;
        float injeck_ua, injeck_ub, injeck_uc;
        float X[3];
        float cur_i_alpha, cur_i_beta;
        float pre_i_alpha, pre_i_beta;
        uint8_t idx;
        float i_alpha_sum_0, i_beta_sum_0;
        float i_alpha_avg_0, i_beta_avg_0;
        float i_alpha_sum_1, i_beta_sum_1;
        float i_alpha_avg_1, i_beta_avg_1;
        float e_theta_mes, e_theta_err;
        float e_theta_hat, e_omega_hat;
        float theta_hat, omega_hat;
        PLL pll;
    } Observer_HFI;
    void observer_hfi_init(Observer_HFI *observer);
    void observer_hfi_update(Observer_HFI *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif