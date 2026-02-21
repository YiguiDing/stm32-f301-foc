#include "motor.h"
#include "delay.h"
#include <stdbool.h>

#include "sensor_i240a2.h"
#include "sensor_as5600.h"

void motor_init(Motor *self, float R, float L, float KV, uint8_t pole_pairs, float power_supply)
{
    // ######################################################################
    motor_set_state(self, Init);
    // ######################################################################
    self->R = R;
    self->L = L;
    self->KV = KV; // rpm/v
    self->pole_pairs = pole_pairs;
    self->KVerps = KV / 60 * pole_pairs; // erps/v
    self->power_supply = power_supply;
    // ######################################################################
    self->target = 0;
    self->sensorless = true;     // 默认无感模式
    self->observer = SMO;        // 观测器类型
    self->sensor = AS5600;       // 传感器类型
    self->use_vf_startup = true; // 是否使用VF启动
    self->control = OPEN_LOOP_VOLTAGE_CTRL;
    self->modulation = SVPWM;
    // ######################################################################
    motor_set_theta_omega(self, 0, 0);
    // ######################################################################
    lpf_init(&self->id_filter, 1 / 100.0f);
    lpf_init(&self->iq_filter, 1 / 100.0f);
    lpf_init(&self->velocity_filter, 1 / 10.0f);
    lpf_init(&self->position_filter, 0.001f);
    // ######################################################################
    self->limit_voltage = 12;
    self->limit_current = 10;
    self->limit_velocity = 500;
    // ######################################################################
    pid_init(&self->pid_id_controller, 1, 0.01, 0, 0, self->limit_voltage, 0.0f);
    pid_init(&self->pid_iq_controller, 1, 0.01, 0, 0, self->limit_voltage, 0.0f);
    pid_init(&self->pid_velocity_controller, 0.01, 0.1, 0, 0, self->limit_current, 0.0f);
    pid_init(&self->pid_position_controller, 5, 2.5, 0, 0, self->limit_velocity, 0.0f);
    // ######################################################################
    sensor_i240a2_init(&self->i240a2);
    sensor_i240a2_align(&self->i240a2, self);
    sensor_as5600_init(&self->as5600);
    sensor_as5600_align(&self->as5600, self);
    // ######################################################################
    observer_smo_init(&self->smo);
    observer_hfi_init(&self->hfi);
    // ######################################################################
    motor_set_state(self, Running);
    // ######################################################################
}
void motor_set_theta_omega(Motor *self, float theta, float omega)
{
    self->theta = theta;
    self->omega = omega;
    float e_theta = theta * self->pole_pairs;
    float e_omega = omega * self->pole_pairs;
    motor_set_e_theta_omega(self, e_theta, e_omega);
}
void motor_set_e_theta_omega(Motor *self, float e_theta, float e_omega)
{
    e_theta = _normalizeAngle(e_theta);
    self->e_theta = e_theta;
    self->e_omega = e_omega;
    self->sin_e_theta = _sin(e_theta);
    self->cos_e_theta = _cos(e_theta);
}
/**
 * 设置abc轴电流
 * - 为防止重复计算三角函数 需要先调用 motor_set_e_theta
 */
void motor_set_abc_current(Motor *self, float Ia, float Ib, float Ic)
{
    self->Ia = Ia;
    self->Ib = Ib;
    self->Ic = Ic;
#define M_SQRT3_3 0.5773502691896257  // sqrt(3)/3
#define M_2SQRT3_3 1.1547005383792515 // 2*sqrt(3)/3
    // 克拉克变换（等幅值形式）
    // Ialpha = Ia
    // Ibeta = (sqrt(3)/3) * Ia + (2*sqrt(3)/3) * Ib
    self->Ialpha = self->Ia;
    self->Ibeta = M_SQRT3_3 * self->Ia + M_2SQRT3_3 * self->Ib;
    // 帕克变换
    // Id = cos(θ) * Ialpha + sin(θ) * Ibeta
    // Id = -sin(θ) * Ialpha + cos(θ) * Ibeta
    self->Id = self->cos_e_theta * self->Ialpha + self->sin_e_theta * self->Ibeta;
    self->Iq = -self->sin_e_theta * self->Ialpha + self->cos_e_theta * self->Ibeta;
}
/**
 * 设置dq轴电压
 * - 为防止重复计算三角函数 需要先调用 motor_set_e_theta
 * - 对应 motor_set_abc_current 的功能，将dq电压转换为abc电压和占空比
 */
void motor_set_dq_voltage(Motor *self, float Ud, float Uq)
{
    float theta = self->e_theta;
    if (Ud != 0)
    {
        // 合成新的矢量
        theta = _normalizeAngle(theta - _atan2(Ud, Uq));
        Uq = _sqrt(Ud * Ud + Uq * Uq);
        Ud = 0;
    }
    self->Ud = Ud + self->hfi.Ud_injeck;
    self->Uq = Uq;

    // 帕克逆变换
    // Ualpha = cos(θ) * Ud - sin(θ) * Uq
    // Ubeta = sin(θ) * Ud + cos(θ) * Uq
    self->Ualpha = self->cos_e_theta * Ud - self->sin_e_theta * Uq;
    self->Ubeta = self->sin_e_theta * Ud + self->cos_e_theta * Uq;

    switch (self->modulation)
    {
    case SPWM: // 克拉克逆变换
    {
#define M_SQRT3_2 0.8660254037844386 // (sqrt(3)/2)

        // 克拉克逆变换(等赋值形式)
        // Ua = Ualpha
        // Ub = -0.5 * Ualpha + (sqrt(3)/2) * Ubeta
        // Uc = -0.5 * Ualpha - (sqrt(3)/2) * Ubeta = -(Ua + Ub) // 基尔霍夫电压电流定律
        self->Ua = self->Ualpha;
        self->Ub = -0.5f * self->Ualpha + M_SQRT3_2 * self->Ubeta;
        self->Uc = -(self->Ua + self->Ub);

        // 计算PWM占空比
        // [-power_supply,+power_supply] => [-0.5,+0.5] => [0,1]
        self->Ta = self->Ua / self->power_supply * 0.5f + 0.5f;
        self->Tb = self->Ub / self->power_supply * 0.5f + 0.5f;
        self->Tc = self->Uc / self->power_supply * 0.5f + 0.5f;
    }
    break;
    case SVPWM:
    {
#define M_PI_3 1.0471975511965979 // 60° = PI/3

        int sector = theta / M_PI_3;
        float alpha = theta - sector * M_PI_3;

        float T1 = Uq / self->power_supply * _sin(M_PI_3 - alpha);
        float T2 = Uq / self->power_supply * _sin(alpha);
        float T0 = 1 - T1 - T2;

        float Ta, Tb, Tc;
        switch (sector)
        {
        case 0:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 1:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 4:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
        }

        // 相电压
        // [0,1] => [-0.5, 0.5] => [-power_supply, +power_supply]
        self->Ua = (Ta - 0.5) * self->power_supply;
        self->Ub = (Tb - 0.5) * self->power_supply;
        self->Uc = (Tc - 0.5) * self->power_supply;

        // pwm占空比
        self->Ta = Ta;
        self->Tb = Tb;
        self->Tc = Tc;
    }
    break;
    default:
        break;
    }
}

void motor_sensor_update(Motor *self, float dt)
{
    sensor_i240a2_update(&self->i240a2);
    motor_set_abc_current(self, self->i240a2.Ia_hat, self->i240a2.Ib_hat, self->i240a2.Ic_hat);
    sensor_as5600_update(&self->as5600, self, dt);
    // motor_set_e_theta_omega(self, self->as5600.theta_mes, self->as5600.omega_hat);
}
void motor_observer_update(Motor *self, float dt)
{
    if (self->observer == SMO)
        observer_smo_update(&self->smo, self, dt);
        // motor_set_e_theta_omega(self, self->smo.theta_hat, self->smo.omega_hat);
    else if (self->observer == HFI)
        observer_hfi_update(&self->hfi, self, dt);
}
/**
 * 设置dq轴电压
 * 中频任务
 */
void motor_control_update(Motor *self, float dt)
{
    if (self->state != Running)
        return;
    switch (self->control)
    {
    case OPEN_LOOP_VOLFREQ_CTRL:
    {
        float vf_target = self->target;
        motor_open_loop_voltage_freq_ctrl(self, vf_target, dt);
        break;
    }
    case OPEN_LOOP_VOLTAGE_CTRL:
    {
        float Ud_target = 0;
        float Uq_target = self->target;
        motor_open_loop_voltage_ctrl(self, Ud_target, Uq_target, dt);
        break;
    }
    case CLOSE_LOOP_CURRENT_CTRL:
    {
        float Id_target = 0;
        float Iq_target = self->target;
        motor_close_loop_current_ctrl(self, Id_target, Iq_target, dt);
        break;
    }
    case CLOSE_LOOP_VELOCITY_CTRL:
    {
        float velocity_target = self->target;
        motor_close_loop_velocity_ctrl(self, velocity_target, dt);
        break;
    }
    case CLOSE_LOOP_POSITION_CTRL:
    {
        float position_target = self->target;
        motor_close_loop_position_ctrl(self, position_target, dt);
        break;
    }
    default:
        break;
    }
}
/**
 * pwm驱动更新
 * 高频任务
 */
void motor_driver_update(Motor *self)
{
    dev_set_pwm_duty(self->Ta, self->Tb, self->Tc);
}

void motor_foc_loop(Motor *self, float dt)
{
    motor_sensor_update(self, dt);
    motor_observer_update(self, dt);
    motor_control_update(self, dt);
    motor_driver_update(self);
}
/**
 * 开环电压频率控制(VF启动)
 */
void motor_open_loop_voltage_freq_ctrl(Motor *self, float vf_target, float dt)
{
    float accFactor = vf_target <= 1.0f ? vf_target : expf((vf_target - 1.0f)); // 加速因子
    float voltage = fminf(accFactor, self->power_supply);                       // 电压限幅
    float e_omega = voltage * self->KVerps;                                     // 电角速度 = V * 电机KV值(单位: e_omega/s)
    float e_theta = self->e_theta + e_omega * dt;
    motor_set_e_theta_omega(self, e_theta, e_omega);
    motor_set_dq_voltage(self, 0, voltage);
}
/**
 * 开环电压控制
 */
void motor_open_loop_voltage_ctrl(Motor *self, float Ud_target, float Uq_target, float dt)
{
    if (!self->sensorless)
    {
        // 有感模式
        if (self->sensor == AS5600)
        {
            // 编码器模式,直接使用传感器反馈
            motor_set_e_theta_omega(self, self->as5600.theta_hat, self->as5600.omega_hat);
            motor_set_dq_voltage(self, Ud_target, Uq_target);
        }
        // else if (其他传感器类型)
        // {
        // }
        return;
    }
    else
    {
        // 无感模式
        if (self->use_vf_startup)
        {
            // 使用VF启动模式
            motor_open_loop_voltage_freq_ctrl(self, Uq_target, dt);
            // 检查SMO是否稳定
            if (observer_smo_check_stabilized(&self->smo, self))
            {
                self->use_vf_startup = false; // 切换到闭环控制
                return;
            }
        }
        else
        {
            // 观测器模式,根据观测器类型选择处理方式
            if (self->observer == SMO)
            {
                // 检查SMO是否不稳定
                if (observer_smo_check_unstabilized(&self->smo, self))
                {
                    self->use_vf_startup = true; // 回退到VF启动模式
                    return;
                }
                // 观测器正常,使用观测器位置
                motor_set_e_theta_omega(self, self->smo.theta_hat, self->smo.omega_hat);
            }
            else if (self->observer == HFI)
            {
                // HFI观测器处理
                // motor_set_e_theta_omega(self, self->hfi.theta_hat, self->hfi.omega_hat);
            }
            motor_set_dq_voltage(self, Ud_target, Uq_target);
        }
    }
}
/**
 * 闭环电流控制
 */
void motor_close_loop_current_ctrl(Motor *self, float Id_target, float Iq_target, float dt)
{
    float Iq_error = Iq_target - lpf_update(&self->iq_filter, self->Iq, dt); // 计算q轴电流误差
    float Uq_target = pid_update(&self->pid_iq_controller, Iq_error, dt);    // 更新q轴电流环pid控制器
    // float Id_error = Id_target - lpf_update(&self->id_filter, self->Id, dt); // 计算d轴电流误差
    // float Ud_target = pid_update(&self->pid_id_controller, Id_error, dt);    // 更新d轴电流环pid控制器
    motor_open_loop_voltage_ctrl(self, 0, Uq_target, dt); // 电流开环控制
}
/**
 * 闭环速度控制
 */
void motor_close_loop_velocity_ctrl(Motor *self, float velocity_target, float dt)
{
    float velocity_error = velocity_target - lpf_update(&self->velocity_filter, self->e_omega, dt); // 计算速度误差
    float Iq_target = pid_update(&self->pid_velocity_controller, velocity_error, dt);               // 更新速度环pid控制器
    motor_close_loop_current_ctrl(self, 0, Iq_target, dt);                                          // 电流开环控制
}
/**
 * 闭环位置控制
 */
void motor_close_loop_position_ctrl(Motor *self, float position_target, float dt)
{
    float position_error = position_target - lpf_update(&self->position_filter, self->theta, dt); // 计算位置误差
    float velocity_target = pid_update(&self->pid_position_controller, position_error, dt);       // 更新位置环pid控制器
    motor_close_loop_velocity_ctrl(self, velocity_target, dt);                                    // 电流开环控制
}
void motor_set_state(Motor *self, MotorState state)
{
    self->state = state;
}
void motor_set_target(Motor *self, float target)
{
    self->target = target;
}
void motor_set_control(Motor *self, ControlType control)
{
    self->control = control;
}
void motor_set_modulation(Motor *self, ModulationType modulation)
{
    self->modulation = modulation;
}