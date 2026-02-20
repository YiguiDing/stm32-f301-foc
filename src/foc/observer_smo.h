#ifndef __OBSERVER_SMO_H__
#define __OBSERVER_SMO_H__

#include <stdint.h>
#include <stdbool.h>
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
        float Ialpha_hat, Ibeta_hat;
        float Ealpha_hat, Ebeta_hat;
        float Hgain, delta;
        float theta_raw, theta_hat, omega_hat;
        LowpassFilter lpf_omega;
        PLL pll;
        uint16_t stabilized_cnt; // 稳定计数器
    } Observer_SMO;
    void observer_smo_init(Observer_SMO *observer);
    void observer_smo_update(Observer_SMO *self, Motor *motor, float dt);

    // 检查观测器是否连续n次稳定
    bool observer_smo_check_stabilized(Observer_SMO *self, Motor *motor);
    // 检查观测器是否连续n次不稳定
    bool observer_smo_check_unstabilized(Observer_SMO *self, Motor *motor);
#ifdef __cplusplus
}
#endif

#endif