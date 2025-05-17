#include "pll.h"
#include "foc_math.h"

void pll_init(PLL *pll, float Ts, float Kp, float Ki)
{
    pll->theta = 0;
    pll->omega = 0;
    lowpass_filter_init(&pll->filter, Ts);
    pid_controller_init(&pll->pid, Kp, Ki, 0, 0, 0);
}

float pll_phrase_diff(float theta, float target)
{

    float diff = _normalizeAngle(target) - _normalizeAngle(theta);
    if (diff > M_PI)
        diff -= M_TWOPI;
    else if (diff < -M_PI)
        diff += M_TWOPI;
    return diff;
}

float pll_update(PLL *pll, float target, float dt)
{
    // 相位差   = sin(theta)*cos(target) - cos(theta)*sin(target)     ∈ [-1,+1]
    //          ≈ sin(theta - target)                                ∈ [-1,+1]
    //          ≈ _normalizeAngle(theta - target) - M_PI             ∈ [-M_PI,+M_PI]
    // 角速度 = PI控制器(lpf低通滤波器(鉴相器))
    // 角度 = 角速度 * dt
    // float phase_diff = _cos(pll->theta) * _sin(target) - _sin(pll->theta) * _cos(target);
    // float phase_diff = _sin(_normalizeAngle(pll->theta - target - M_PI));
    // float phase_diff = _normalizeAngle(pll->theta - target) - M_PI;
    float phase_diff = pll_phrase_diff(pll->theta, target);
    pll->omega = pid_controller_update(&pll->pid, lowpass_filter_update(&pll->filter, phase_diff, dt), dt);
    pll->theta += pll->omega * dt;
    return pll->theta;
}

float pll_update_pd(PLL *pll, float phase_diff, float dt)
{
    // 角速度 = PI控制器(lpf低通滤波器(鉴相器))
    // 角度 = 角速度 * dt
    pll->omega = pid_controller_update(&pll->pid, lowpass_filter_update(&pll->filter, phase_diff, dt), dt);
    pll->theta += pll->omega * dt;
    return pll->theta;
}