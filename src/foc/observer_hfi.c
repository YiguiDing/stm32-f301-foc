#include "observer_hfi.h"
#include "motor.h"
#include "foc_math.h"

void observer_hfi_init(Observer_HFI *self)
{
    self->sign = 1;
    self->Ud_injeck = 0;
    self->Ialpha_prev = 0;
    self->Ibeta_prev = 0;
    self->theta_mes = 0;
    self->theta_hat = 0;
    self->omega_hat = 0;
    self->X[0] = 0;
    self->X[1] = 0;
    self->X[2] = 0;
    // pll_init(&self->pll, 0.000011f, 100, 25);
    // pll_init(&self->pll, 0.000001f, 200, 150); // pwm_freq = 18khz => 5.5e-5
    // pll_init(&self->pll, 0.000001f, 250, 200); // pwm_freq = 18khz => 5.5e-5
    // pll_init(&self->pll, 0.000500f, 200, 50);  // pwm_freq = 18khz => 5.5e-5
}

void observer_hfi_update(Observer_HFI *self, Motor *motor, float dt)
{
    // 计算角度
    float Ialpha_h = self->Ialpha_prev - motor->Ialpha; // alpha轴高频电流
    float Ibeta_h = self->Ibeta_prev - motor->Ibeta;    // beta轴高频电流

    self->theta_mes = _normalizeAngle(_atan2(Ibeta_h, Ialpha_h)); // 直接计算 [-PI,+PI] => [0,2PI]
    // 注入到d轴
    self->Ud_injeck = self->sign * 0.08 * motor->power_supply;

    // 相位差   = sin(theta)*cos(target) - cos(theta)*sin(target)     ∈ [-1,+1]
    //          ≈ sin(theta - target)                                ∈ [-1,+1]
    //          ≈ _normalizeAngle(theta - target) - M_PI             ∈ [-M_PI,+M_PI]
    // 角速度 = PI控制器(lpf低通滤波器(鉴相器))
    // 角度 = 角速度 * dt
    float diff = _phrase_diff(self->theta_mes, self->pll.theta_hat);
    pll_update(&self->pll, diff, dt);

    self->theta_hat = self->pll.theta_hat;
    self->omega_hat = self->pll.omega_hat;

    // 保存电流
    self->Ialpha_prev = motor->Ialpha; //
    self->Ibeta_prev = motor->Ibeta;   //
    self->sign = -self->sign;

    return;

    // rk4
    // runge_kutta_4(position_observer, self->e_theta_mes, self->X, 3, dt);
    // self->e_theta_hat = self->X[0];
    // self->e_omega_hat = self->X[1];

    // motor->theta = self->theta_hat;
    // motor->omega = self->omega_hat;
}

/**
 * 位置观测器动态方程
 *
 * 《High Bandwidth Sensorless Algorithm for AC Machines Based on Square-wave Type Voltage Injection》--Young-doo Yoon
 * http://eepel.snu.ac.kr/wordpress/files/papers/conference/international/20091008_121712_High%20Bandwidth%20Sensorless%20Algorithm%20for%20AC%20Machines%20Based%20on%20Square-wave%20Type%20Voltage%20Injection/High%20Bandwidth_Young%20doo.pdf
 *
 * 《基于无滤波器方波信号注入的永磁同步电机初始位置检测方法》-- 张国强
 * https://dgjsxb.ces-transaction.com/CN/abstract/abstract4574.shtml
 */
void position_observer(float *X_dot, float *X, float theta)
{
#define Gain_0 30
#define Gain_1 900
#define Gain_2 1500
#define Te 1     // 电磁转矩
#define Jeq 0.06 // 转动惯量 PMSM_SHAFT_INERTIA 0.06           // delta =3, 4,  10
#define PP 7     // 极对数
    float diff = _phrase_diff(theta, X[0]);
    X_dot[0] = Gain_0 * diff + X[1];
    X_dot[1] = Gain_1 * diff + X[2] + (PP * Te / Jeq);
    X_dot[2] = Gain_2 * diff;
}
void runge_kutta_4(
    void (*ode)(float *X_dot, float *X, float theta), //
    float *X,                                         //
    float theta,                                      //
    uint8_t n,                                        //
    float dt                                          //
)
{
    float k1[n], k2[n], k3[n], k4[n];
    float _X[n], _X_dot[n];

    ode(_X_dot, X, theta);
    for (uint8_t i = 0; i < n; ++i)
    {
        k1[i] = _X_dot[i] * dt;
        _X[i] = X[i] + k1[i] / 2;
    }

    ode(_X_dot, _X, theta);
    for (uint8_t i = 0; i < n; ++i)
    {
        k2[i] = _X_dot[i] * dt;
        _X[i] = X[i] + k2[i] / 2;
    }

    ode(_X_dot, _X, theta);
    for (uint8_t i = 0; i < n; ++i)
    {
        k3[i] = _X_dot[i] * dt;
        _X[i] = X[i] + k3[i];
    }

    ode(_X_dot, _X, theta);
    for (uint8_t i = 0; i < n; ++i)
    {
        k4[i] = _X_dot[i] * dt;
        X[i] = X[i] + (k1[i] + 2 * (k2[i] + k3[i]) + k4[i]) / 6.0f;
    }
}
