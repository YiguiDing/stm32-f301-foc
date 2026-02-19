#ifndef __PLL_H__
#define __PLL_H__

#include "foc_math.h"
#include "lpf.h"
#include "pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float theta_hat; // 估计角度
        float omega_hat; // 估计角速度
        LowpassFilter lpf;
        PidController pid;
    } PLL;

    void pll_init(PLL *pll, float alpha, float Kp, float Ki);
    float pll_update(PLL *pll, float theta, float dt);
    float pll_update_pd(PLL *pll, float phase_diff, float dt);

#ifdef __cplusplus
}
#endif

#endif