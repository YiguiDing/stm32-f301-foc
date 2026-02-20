#include "observer_smo.h"
#include "motor.h"

void observer_smo_init(Observer_SMO *self)
{
    self->Ialpha_hat = 0.0f;
    self->Ibeta_hat = 0.0f;
    self->Ealpha_hat = 0.0f;
    self->Ebeta_hat = 0.0f;
    self->Hgain = 0.05f; // 增大滑膜增益
    self->delta = 0.5f;  // 增大饱和阈值，避免频繁饱和
    self->theta_raw = 0.0f;
    self->theta_hat = 0.0f;
    self->omega_hat = 0.0f;

    // lpf_init(&self->lpf_omega, 1 / 100.0f);
    pll_init(&self->pll, 0 / 500.0f, 500.0f, 10.0f);
}

float smo_sat(float x, float delta)
{
    if (x > delta)
        return 1;
    if (x < -delta)
        return -1;
    return x / delta;
}

void observer_smo_update(Observer_SMO *self, Motor *motor, float Ts)
{
    // 电机参数
    float invL = 1.0f / motor->L;
    float R_invL = motor->R * invL;

    // 电流观测器离散化：采用前向欧拉法
    // I_hat[k+1] = I_hat[k] + Ts * (-(R/L)*I_hat[k] + (1/L)*(U - E_hat))
    self->Ialpha_hat += Ts * (-R_invL * self->Ialpha_hat + invL * (motor->Ualpha - self->Ealpha_hat));
    self->Ibeta_hat += Ts * (-R_invL * self->Ibeta_hat + invL * (motor->Ubeta - self->Ebeta_hat));

    // 更新反电动势估计
    self->Ealpha_hat = self->Hgain * smo_sat(self->Ialpha_hat - motor->Ialpha, self->delta);
    self->Ebeta_hat = self->Hgain * smo_sat(self->Ibeta_hat - motor->Ibeta, self->delta);

    // 1 直接计算 噪声过大 能观测角度，但无法闭环
    // float theta_raw = -_atan2(self->Ealpha_hat, self->Ebeta_hat);
    // float omega_raw = _phrase_diff(theta_raw, self->theta_hat) / Ts;
    // self->theta_hat = _normalizeAngle(theta_raw);
    // self->omega_hat = lpf_update(&self->lpf_omega, omega_raw, Ts);

    // 2 滤波计算 能观测角度，但无法闭环
    // static float Ealpha = 0, Ebeta = 0;
    // Ealpha = 0.9 * self->Ealpha_hat + 0.1 * Ealpha;
    // Ebeta = 0.9 * self->Ebeta_hat + 0.1 * Ebeta;
    // float theta_raw = -_atan2(Ealpha, Ebeta);
    // float omega_raw = _phrase_diff(theta_raw, self->theta_hat) / Ts;
    // self->theta_hat = _normalizeAngle(theta_raw);
    // self->omega_hat = lpf_update(&self->lpf_omega, omega_raw, Ts);

    // 3 直接计算 + 锁相环 能观测角度，可以闭环
    self->theta_raw = _normalizeAngle(-_atan2(self->Ealpha_hat, self->Ebeta_hat));
    float diff = _phrase_diff(self->theta_raw, self->pll.theta_hat);
    pll_update(&self->pll, diff, Ts);
    self->theta_hat = _normalizeAngle(self->pll.theta_hat + M_PI_2); // TODO: 解释这里为什么相位偏移90度
    self->omega_hat = self->pll.omega_hat;
    // 3 滤波计算 + 锁相环 实际测试可以闭环，和上面区别不大
    // static float Ealpha = 0, Ebeta = 0;
    // Ealpha = 0.9 * self->Ealpha_hat + 0.1 * Ealpha;
    // Ebeta = 0.9 * self->Ebeta_hat + 0.1 * Ebeta;
    // self->theta_raw = _normalizeAngle(-_atan2(Ealpha, Ebeta));
    // float diff = _phrase_diff(self->theta_raw, self->pll.theta_hat);
    // pll_update(&self->pll, diff, Ts);
    // self->theta_hat = self->pll.theta_hat;
    // self->omega_hat = self->pll.omega_hat;

    // 4 利用d轴反电动势应为0作为锁相环的鉴相器 实际测试可以闭环，但是需要再算一次sin和cos,另外依然存在锁相环角度和实际角度相差90°的问题。
    // self->theta_raw = _normalizeAngle(-_atan2(self->Ealpha_hat, self->Ebeta_hat));
    // float cos_e_theta = _cos(self->pll.theta_hat);
    // float sin_e_theta = _sin(self->pll.theta_hat);
    // // 帕克变换
    // float Ed_hat = cos_e_theta * self->Ealpha_hat + sin_e_theta * self->Ebeta_hat;
    // float Eq_hat = -sin_e_theta * self->Ealpha_hat + cos_e_theta * self->Ebeta_hat;
    // float Ed_err = 0 - Ed_hat;
    // pll_update(&self->pll, 10 * Ed_err, Ts);
    // self->theta_hat = self->pll.theta_hat;
    // self->omega_hat = self->pll.omega_hat;
}