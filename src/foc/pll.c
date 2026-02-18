#include "pll.h"
#include "foc.h"

void pll_init(PLL *pll, float alpha, float Kp, float Ki, float omega_limit)
{
    pll->theta_hat = 0;
    pll->omega_hat = 0;
    lpf_init(&pll->lpf, alpha);
    pid_init(&pll->pid, Kp, Ki, 0, omega_limit, 0, 0);
}

float pll_update(PLL *pll, float phrase_diff, float dt)
{
    // 角速度 = lpf低通滤波器(PI控制器(相位差))
    // 角度 = 角速度 * dt
    pll->omega_hat = lpf_update(&pll->lpf, pid_update(&pll->pid, phrase_diff, dt), dt);
    pll->theta_hat = _normalizeAngle(pll->theta_hat + pll->omega_hat * dt);
    return pll->theta_hat;
}