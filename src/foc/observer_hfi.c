#include "observer_hfi.h"
#include "motor.h"

void observer_hfi_init(Observer_HFI *self)
{
    self->injeck_ud = 0;
    self->X[0] = self->X[1] = self->X[2] = 0;
    self->cur_i_alpha = self->cur_i_beta = 0;
    self->pre_i_alpha = self->pre_i_beta = 0;
    self->e_theta_mes = self->e_theta_err = 0;
    self->e_theta_hat = self->e_omega_hat = 0;
    // pll_init(&self->pll, 0.000011f, 100, 25);
    // pll_init(&self->pll, 0.000001f, 200, 150); // pwm_freq = 18khz => 5.5e-5
    // pll_init(&self->pll, 0.000001f, 250, 200); // pwm_freq = 18khz => 5.5e-5
    // pll_init(&self->pll, 0.000500f, 200, 50);  // pwm_freq = 18khz => 5.5e-5
}

float phrase_diff(float theta, float target)
{

    float diff = _normalizeAngle(target) - _normalizeAngle(theta);
    if (diff > M_PI)
        diff -= M_TWOPI;
    else if (diff < -M_PI)
        diff += M_TWOPI;
    return diff; // diff ∈ [-M_PI,+M_PI]
}
void observer_hfi_update(Observer_HFI *self, Motor *motor, float dt)
{
    // 获取电流
    self->cur_i_alpha = motor->Ialpha;
    self->cur_i_beta = motor->Ibeta;
    // 计算角度
    int8_t sign = self->injeck_ud > 0 ? +1 : -1;                      // 注入电压极性
    float i_alpha_h = sign * (self->cur_i_alpha - self->pre_i_alpha); // alpha轴高频电流
    float i_beta_h = sign * (self->cur_i_beta - self->pre_i_beta);    // beta轴高频电流
    self->e_theta_mes = _normalizeAngle(_atan2(i_beta_h, i_alpha_h)); // 直接计算 [-PI,+PI] => [0,2PI]
    // for next time.
    self->pre_i_alpha = self->cur_i_alpha;
    self->pre_i_beta = self->cur_i_beta;

    // 注入到d轴
    // self->injeck_ud = -sign * (motor->Uq * 0.5 + 0.1 * motor->power_supply);
    self->injeck_ud = -sign * (0.08 * motor->power_supply);

    // 锁相环
    if (!isnanf(self->e_theta_mes))
    {
        // 相位差   = sin(theta)*cos(target) - cos(theta)*sin(target)     ∈ [-1,+1]
        //          ≈ sin(theta - target)                                ∈ [-1,+1]
        //          ≈ _normalizeAngle(theta - target) - M_PI             ∈ [-M_PI,+M_PI]
        // 角速度 = PI控制器(lpf低通滤波器(鉴相器))
        // 角度 = 角速度 * dt
        // float phase_diff = _cos(pll->theta) * _sin(target) - _sin(pll->theta) * _cos(target);
        // float phase_diff = _sin(_normalizeAngle(pll->theta - target - M_PI));
        // float phase_diff = _normalizeAngle(pll->theta - target) - M_PI;
        pll_update(&self->pll, phrase_diff(self->e_theta_mes, self->pll.theta_hat), dt);
    }

    self->e_theta_hat = self->pll.theta_hat;
    self->e_omega_hat = self->pll.omega_hat;
    self->theta_hat = self->e_theta_hat / motor->pole_pairs;
    self->omega_hat = self->e_omega_hat / motor->pole_pairs;

    // rk4
    // runge_kutta_4(position_observer, self->e_theta_mes, self->X, 3, dt);
    // self->e_theta_hat = self->X[0];
    // self->e_omega_hat = self->X[1];

    motor->Ud += self->injeck_ud;

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
void position_observer(float theta, float *X, float *X_dot)
{
#define Gain_0 30
#define Gain_1 900
#define Gain_2 1500
#define Te 1     // 电磁转矩
#define Jeq 0.06 // 转动惯量 PMSM_SHAFT_INERTIA 0.06           // delta =3, 4,  10
#define PP 7     // 极对数
    float diff = phrase_diff(theta, X[0]);
    X_dot[0] = Gain_0 * diff + X[1];
    X_dot[1] = Gain_1 * diff + X[2] + (PP * Te / Jeq);
    X_dot[2] = Gain_2 * diff;
}
void runge_kutta_4(
    void (*ode_function)(float input, float *X, float *X_dot), //
    float input, float *X,                                     //
    uint8_t n,                                                 //
    float dt                                                   //
)
{
    float k1[n], k2[n], k3[n], k4[n]; // C89/C90标准是不支持可变长度数组的, C99标准引入了可变长度数组（VLA）的特性，允许数组的长度在运行时确定。
    float temp_X[n], temp_X_dot[n];

    ode_function(input, X, temp_X_dot);
    for (uint8_t i = 0; i < n; ++i)
    {
        k1[i] = temp_X_dot[i] * dt;
        temp_X[i] = X[i] + k1[i] / 2;
    }

    ode_function(input, temp_X, temp_X_dot);
    for (uint8_t i = 0; i < n; ++i)
    {
        k2[i] = temp_X_dot[i] * dt;
        temp_X[i] = X[i] + k2[i] / 2;
    }

    ode_function(input, temp_X, temp_X_dot);
    for (uint8_t i = 0; i < n; ++i)
    {
        k3[i] = temp_X_dot[i] * dt;
        temp_X[i] = X[i] + k3[i];
    }

    ode_function(input, temp_X, temp_X_dot);
    for (uint8_t i = 0; i < n; ++i)
    {
        k4[i] = temp_X_dot[i] * dt;
        X[i] = X[i] + (k1[i] + 2 * (k2[i] + k3[i]) + k4[i]) / 6.0f;
    }
}
