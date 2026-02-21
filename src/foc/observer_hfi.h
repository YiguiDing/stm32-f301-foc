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
        float Ialpha_h, Ibeta_h; // 高频电流响应
        float theta_raw, theta_hat, omega_hat;
        float X[3];
        PLL pll;
    } Observer_HFI;
    void observer_hfi_init(Observer_HFI *observer);
    // PWM中断调用：计算注入电压(翻转符号)
    void observer_hfi_on_pwm_update(Observer_HFI *self, float power_supply);
    // ADC中断调用：计算高频电流响应
    void observer_hfi_on_adc_update(Observer_HFI *self, float Ialpha, float Ibeta);
    // Observer任务调用：根据高频电流计算角度
    void observer_hfi_update(Observer_HFI *self, float dt);
#ifdef __cplusplus
}
#endif

#endif