#include "observer_smo.h"
#include "motor.h"

void observer_smo_init(Observer_SMO *self)
{
    self->Ialpha_hat = 0.0f;
    self->Ibeta_hat = 0.0f;
    self->Ealpha_hat = 0.0f;
    self->Ebeta_hat = 0.0f;
    self->Hgain = 0.3f;
    self->delta = 0.5f;

    self->theta_hat = 0.0f;
    self->omega_hat = 0.0f;

    // lpf_init(&self->lpf_omega, 1 / 100.0f);
    pll_init(&self->pll, 0, 200.0f, 0.0f, 0.0f);
}
float smo_sat(float x, float delta)
{
    if (x > delta)
        return 1;
    if (x < -delta)
        return -1;
    return x / delta;
}
float smo_phrase_diff(float target, float theta)
{
    float diff = _normalizeAngle(target) - _normalizeAngle(theta);
    if (diff > M_PI)
        diff -= M_TWOPI;
    else if (diff < -M_PI)
        diff += M_TWOPI;
    return diff; // diff ∈ [-M_PI,+M_PI]
}
void observer_smo_update(Observer_SMO *self, Motor *motor, float Ts)
{
    // 电机参数
    float invL = 1.0f / motor->L;
    float R_invL = motor->R * invL;

    // 限制最大负反馈以防止数值发散
    float damping_factor = 1.0f;
    if (R_invL * Ts > 1.0f)
        damping_factor = 0.5f / (R_invL * Ts); // 当 R/L 过大时，限制离散化步长

    // 电流观测器离散化：采用前向欧拉法
    // I_hat[k+1] = I_hat[k] + Ts * (-(R/L)*I_hat[k] + (1/L)*(U - E_hat))
    self->Ialpha_hat += damping_factor * Ts * (-R_invL * self->Ialpha_hat + invL * (motor->Ualpha - self->Ealpha_hat));
    self->Ibeta_hat += damping_factor * Ts * (-R_invL * self->Ibeta_hat + invL * (motor->Ubeta - self->Ebeta_hat));

    // 更新反电动势估计
    self->Ealpha_hat = self->Hgain * smo_sat(self->Ialpha_hat - motor->Ialpha, self->delta);
    self->Ebeta_hat = self->Hgain * smo_sat(self->Ibeta_hat - motor->Ibeta, self->delta);

    // 1 直接计算 噪声过大 能观测角度，但无法闭环
    // float theta_raw = -_atan2(self->Ealpha_hat, self->Ebeta_hat);
    // float omega_raw = smo_phrase_diff(theta_raw, self->theta_hat) / Ts;
    // self->theta_hat = _normalizeAngle(theta_raw);
    // self->omega_hat = lpf_update(&self->lpf_omega, omega_raw, Ts);

    // 2 滤波计算 能观测角度，但无法闭环
    // static float Ealpha = 0, Ebeta = 0;
    // Ealpha = 0.9 * self->Ealpha_hat + 0.1 * Ealpha;
    // Ebeta = 0.9 * self->Ebeta_hat + 0.1 * Ebeta;
    // float theta_raw = -_atan2(Ealpha, Ebeta);
    // float omega_raw = smo_phrase_diff(theta_raw, self->theta_hat) / Ts;
    // self->theta_hat = _normalizeAngle(theta_raw);
    // self->omega_hat = lpf_update(&self->lpf_omega, omega_raw, Ts);

    // 3 直接计算 + 锁相环 能观测角度，可以闭环
    self->theta_raw = _normalizeAngle(-_atan2(self->Ealpha_hat, self->Ebeta_hat));
    float diff = smo_phrase_diff(self->theta_raw, self->pll.theta_hat);
    pll_update(&self->pll, diff, Ts);
    self->theta_hat = self->pll.theta_hat;
    self->omega_hat = self->pll.omega_hat;

    // 3 滤波计算 + 锁相环 未实现测试
    // static float Ealpha = 0, Ebeta = 0;
    // Ealpha = 0.9 * self->Ealpha_hat + 0.1 * Ealpha;
    // Ebeta = 0.9 * self->Ebeta_hat + 0.1 * Ebeta;
    // float theta_raw = -_atan2(self->Ealpha_hat, self->Ebeta_hat);
    // float diff = smo_phrase_diff(theta_raw, self->pll.theta_hat);
    // pll_update(&self->pll, diff, Ts);
    // self->theta_hat = self->pll.theta_hat;
    // self->omega_hat = self->pll.omega_hat;

    // 4 未实现测试
    // self->theta_raw = _normalizeAngle(-_atan2(self->Ealpha_hat, self->Ebeta_hat));
    // float E_d = (motor->cos_e_theta * self->Ealpha_hat + motor->sin_e_theta * self->Ebeta_hat);
    // float E_d_err = 0 - E_d;
    // pll_update(&self->pll, E_d_err, Ts);
    // self->theta_hat = -self->pll.theta_hat;
    // self->omega_hat = -self->pll.omega_hat;
}