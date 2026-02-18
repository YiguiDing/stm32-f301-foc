#include "motor.h"
#include "delay.h"

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
    self->observer = AS5600;
    self->control = OPEN_LOOP_VOLTAGE_CTRL;
    self->modulation = SVPWM;
    // ######################################################################
    motor_set_theta_omega(self, 0, 0);
    // ######################################################################
    lpf_init(&self->id_filter, 0.0001f);
    lpf_init(&self->iq_filter, 0.0001f);
    lpf_init(&self->velocity_filter, 0.01f);
    lpf_init(&self->position_filter, 0.001f);
    // ######################################################################
    self->limit_voltage = 12;
    self->limit_current = 10;
    self->limit_velocity = 500;
    // ######################################################################
    pid_init(&self->pid_id_controller, 2, 0.5, 0, self->limit_voltage, 0.0f);
    pid_init(&self->pid_iq_controller, 2, 0.5, 0, self->limit_voltage, 0.0f);
    pid_init(&self->pid_velocity_controller, 0.1, 0.05, 0, self->limit_current, 0.0f);
    pid_init(&self->pid_position_controller, 5, 2.5, 0, self->limit_velocity, 0.0f);
    // ######################################################################
    sensor_i240a2_init(&self->i240a2);
    sensor_i240a2_align(&self->i240a2, self);
    sensor_as5600_init(&self->as5600);
    sensor_as5600_align(&self->as5600, self);
    // ######################################################################
    observer_smo_init(&self->smo);
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
void motor_set_phrase_current(Motor *self, float Ia, float Ib, float Ic)
{
    // #define M_SQRT3 1.73205080757f // sqrt(3)
    self->Ia = Ia;
    self->Ib = Ib;
    self->Ic = Ic;
    // 克拉克变换
    self->Ialpha = self->Ia; // 等幅值形式
    self->Ibeta = M_SQRT3 / 3 * self->Ia + 2 * M_SQRT3 / 3 * self->Ib;
    // 帕克变换
    self->Id = self->cos_e_theta * self->Ialpha + self->sin_e_theta * self->Ibeta;
    self->Iq = -self->sin_e_theta * self->Ialpha + self->cos_e_theta * self->Ibeta;
}
/**
 * 设置dq轴电压
 * - 需要先调用 motor_set_e_theta
 */
void motor_set_dq_voltage(Motor *self, float Ud, float Uq, float e_theta)
{
    self->Ud = Ud;
    self->Uq = Uq;
    // 帕克逆变换
    float cos_e_theta = _cos(e_theta);
    float sin_e_theta = _sin(e_theta);
    self->Ualpha = cos_e_theta * Ud - sin_e_theta * Uq;
    self->Ubeta = sin_e_theta * Ud + cos_e_theta * Uq;

    switch (self->modulation)
    {
    case SPWM: // 帕克逆变换+克拉克逆变换 耗时13us
    {
        // 克拉克逆变换
        self->Ua = self->Ualpha; // 等赋值形式
        self->Ub = -0.5f * self->Ualpha + 0.5f * M_SQRT3 * self->Ubeta;
        self->Uc = -(self->Ua + self->Ub); // 基尔霍夫电压电流定律
        // 设置相电压
        // [-power_supply,+power_supply] => [-0.5,+0.5] => [0,1]
        self->Ta = self->Ua / self->power_supply * 0.5f + 0.5f;
        self->Tb = self->Ub / self->power_supply * 0.5f + 0.5f;
        self->Tc = self->Uc / self->power_supply * 0.5f + 0.5f;
    }
    break;
    case SVPWM:
    {
#define M_PI_3 1.0471975512f // pi/3

        int sector = e_theta / M_PI_3;
        float alpha = e_theta - sector * M_PI_3;

        float H = self->Uq / self->power_supply;

        float T1 = H * _sin(M_PI_3 - alpha);
        float T2 = H * _sin(alpha);
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

        self->Ua = (Ta - 0.5) * self->power_supply;
        self->Ub = (Tb - 0.5) * self->power_supply;
        self->Uc = (Tc - 0.5) * self->power_supply;

        self->Ta = Ta;
        self->Tb = Tb;
        self->Tc = Tc;
    }
    break;
    default:
        break;
    }
}

void motor_observer_update(Motor *self, float dt)
{
    sensor_i240a2_update(&self->i240a2);
    motor_set_phrase_current(self, self->i240a2.Ia_hat, self->i240a2.Ib_hat, self->i240a2.Ic_hat);

    observer_smo_update(&self->smo, self, dt);
    // motor_set_e_theta_omega(self, self->smo.theta_hat, self->smo.omega_hat);

    // sensor_as5600_update(&self->as5600, self, dt);
    // motor_set_theta_omega(self, self->as5600.theta_mes, self->as5600.omega_hat);
}
/**
 * 设置dq轴电压
 * 中频任务
 */
void motor_ctontrol_update(Motor *self, float dt)
{
    if (self->state != Running)
        return;

    switch (self->control)
    {
    case OPEN_LOOP_VOLFREQ_CTRL:
    {
        float vf_target = self->target;
        motor_open_loop_voltage_freq_ctrl(self, vf_target, dt);
        if (self->target > 5)
            self->control = OPEN_LOOP_VOLTAGE_CTRL;
        break;
    }
    case OPEN_LOOP_VOLTAGE_CTRL:
    {
        float Ud_target = 0;
        float Uq_target = self->target;
        // self->e_theta = _normalizeAngle(self->smo.theta_hat);
        self->e_theta = _normalizeAngle(self->smo.theta_hat + M_PI_2);
        motor_open_loop_voltage_ctrl(self, Ud_target, Uq_target, dt);
        if (self->target < 5)
            self->control = OPEN_LOOP_VOLFREQ_CTRL;
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
    motor_observer_update(self, dt);
    motor_ctontrol_update(self, dt);
    motor_driver_update(self);
}
/**
 * 开环电压频率控制
 */
void motor_open_loop_voltage_freq_ctrl(Motor *self, float vf_target, float dt)
{
    float voltage_target = vf_target;
    self->e_omega = voltage_target * 15 * self->KVerps; // 20倍kv值
    self->e_theta = _normalizeAngle(self->e_theta += self->e_omega * dt);
    motor_set_dq_voltage(self, 0, voltage_target, self->e_theta);
}
/**
 * 开环电压控制
 */
void motor_open_loop_voltage_ctrl(Motor *self, float Ud_target, float Uq_target, float dt)
{
    motor_set_dq_voltage(self, Ud_target, Uq_target, self->e_theta);
}
/**
 * 闭环电流控制
 */
void motor_close_loop_current_ctrl(Motor *self, float Id_target, float Iq_target, float dt)
{
    float Id_error = Id_target - lpf_update(&self->id_filter, self->Id, dt); // 计算d轴电流误差
    float Iq_error = Iq_target - lpf_update(&self->iq_filter, self->Iq, dt); // 计算q轴电流误差
    float Ud_target = pid_update(&self->pid_id_controller, Id_error, dt);    // 更新d轴电流环pid控制器
    float Uq_target = pid_update(&self->pid_iq_controller, Iq_error, dt);    // 更新q轴电流环pid控制器
    motor_set_dq_voltage(self, Ud_target, Uq_target, self->e_theta);         // 设置dq轴电压
}
/**
 * 闭环速度控制
 */
void motor_close_loop_velocity_ctrl(Motor *self, float velocity_target, float dt)
{
    float velocity_error = velocity_target - lpf_update(&self->velocity_filter, self->omega, dt); // 计算速度误差
    float Iq_target = pid_update(&self->pid_velocity_controller, velocity_error, dt);             // 更新速度环pid控制器
    motor_close_loop_current_ctrl(self, 0, Iq_target, dt);                                        // 电流开环控制
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