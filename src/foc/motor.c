#include "motor.h"
#include "delay.h"

void motor_init(Motor *motor, uint8_t pole_pairs, float power_supply, uint8_t ch_ia, uint8_t ch_ib, uint8_t ch_theta)
{
    // ######################################################################
    motor->pole_pairs = pole_pairs;
    motor->power_supply = power_supply;
    motor->ch_ia = ch_ia;
    motor->ch_ib = ch_ib;
    // ######################################################################
    motor->target = 0;
    motor->observer = AS5600;
    motor->control = OPEN_LOOP_VOLTAGE_CTRL;
    motor->modulation = SPWM;
    // ######################################################################
    motor_set_theta_omega(motor, 0, 0);
    // ######################################################################
    lowpass_filter_init(&motor->id_filter, 0.0001f);
    lowpass_filter_init(&motor->iq_filter, 0.0001f);
    lowpass_filter_init(&motor->velocity_filter, 0.01f);
    lowpass_filter_init(&motor->position_filter, 0.001f);
    // ######################################################################
    motor->limit_voltage = 12;
    motor->limit_current = 10;
    motor->limit_velocity = 500;
    // ######################################################################
    pid_controller_init(&motor->pid_id_controller, 2, 0.5, 0, motor->limit_voltage, 0.0f);
    pid_controller_init(&motor->pid_iq_controller, 2, 0.5, 0, motor->limit_voltage, 0.0f);
    pid_controller_init(&motor->pid_velocity_controller, 0.1, 0.05, 0, motor->limit_current, 0.0f);
    pid_controller_init(&motor->pid_position_controller, 5, 2.5, 0, motor->limit_velocity, 0.0f);
    // ######################################################################
    observer_fake_init(&motor->fake, 2); // 10°/s
    observer_as5600_init(&motor->as5600, ch_theta);
    observer_as5600_align(&motor->as5600, motor);
    observer_hfi_init(&motor->hfi);
    // ######################################################################
    dev_set_pwm_enable(Bit_SET);
    // ######################################################################
}
void motor_set_theta_omega(Motor *motor, float theta, float omega)
{
    motor->theta = theta;
    motor->omega = omega;
    float e_theta = theta * motor->pole_pairs;
    float e_omega = omega * motor->pole_pairs;
    motor_set_e_theta_omega(motor, e_theta, e_omega);
}
void motor_set_e_theta_omega(Motor *motor, float e_theta, float e_omega)
{
    e_theta = _normalizeAngle(e_theta);
    motor->e_theta = e_theta;
    motor->e_omega = e_omega;
    motor->sin_e_theta = _sin(e_theta);
    motor->cos_e_theta = _cos(e_theta);
}
void motor_set_phrase_current(Motor *motor, float Ia, float Ib, float Ic)
{
    // #define M_SQRT3 1.73205080757f // sqrt(3)
    motor->Ia = Ia;
    motor->Ib = Ib;
    motor->Ic = Ic;
    // 克拉克变换
    motor->Ialpha = motor->Ia; // 等幅值形式
    motor->Ibeta = M_SQRT3 / 3 * motor->Ia + 2 * M_SQRT3 / 3 * motor->Ib;
    // 帕克变换
    motor->Id = motor->cos_e_theta * motor->Ialpha + motor->sin_e_theta * motor->Ibeta;
    motor->Iq = -motor->sin_e_theta * motor->Ialpha + motor->cos_e_theta * motor->Ibeta;
}
/**
 * 设置dq轴电压
 * - 需要先调用 motor_set_e_theta
 */
void motor_set_dq_voltage(Motor *motor, float Ud, float Uq)
{
    motor->Ud = Ud;
    motor->Uq = Uq;

    switch (motor->modulation)
    {
    case SPWM: // 帕克逆变换+克拉克逆变换 耗时13us
    {
#define M_SQRT3_2 0.866025403785 // sqrt(3)/2
        // 帕克逆变换
        motor->Uahpha = motor->cos_e_theta * Ud - motor->sin_e_theta * Uq;
        motor->Ubeta = motor->sin_e_theta * Ud + motor->cos_e_theta * Uq;
        // 克拉克逆变换
        motor->Ua = motor->Uahpha; // 等赋值形式
        motor->Ub = -0.5f * motor->Uahpha + M_SQRT3_2 * motor->Ubeta;
        motor->Uc = -(motor->Ua + motor->Ub); // 基尔霍夫电压电流定律
        // 设置相电压
        // [-power_supply,+power_supply] => [-0.5,+0.5] => [0,1]
        motor->Ta = motor->Ua / motor->power_supply * 0.5f + 0.5f;
        motor->Tb = motor->Ub / motor->power_supply * 0.5f + 0.5f;
        motor->Tc = motor->Uc / motor->power_supply * 0.5f + 0.5f;
    }
    break;
    case SVPWM:
    {
#define M_PI_3 1.0471975512f // pi/3
                             // #define M_SQRT3 1.73205080757f // sqrt(3)

        int sector = floor(motor->e_theta / M_PI_3) + 1;

        float T1 = M_SQRT3 * sinf(sector * M_PI_3 - motor->e_theta) * motor->Uq / motor->power_supply;
        float T2 = M_SQRT3 * sinf(motor->e_theta - (sector - 1.0f) * M_PI_3) * motor->Uq / motor->power_supply;
        float T0 = 1 - T1 - T2;

        float Ta, Tb, Tc;
        switch (sector)
        {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
        }

        motor->Ua = (Ta - 0.5) * motor->power_supply;
        motor->Ub = (Tb - 0.5) * motor->power_supply;
        motor->Uc = (Tc - 0.5) * motor->power_supply;

        motor->Ta = Ta;
        motor->Tb = Tb;
        motor->Tc = Tc;
    }
    break;
    default:
        break;
    }
    dev_set_pwm_duty(motor->Ta, motor->Tb, motor->Tc);
}

void motor_current_observer_update(Motor *motor, float dt)
{
// ########################################################
//                        电流观测
// ########################################################
#define Ia_OFFSET +0.11
#define Ib_OFFSET +0.005

    float Ia = (dev_red_adc(motor->ch_ia) - 0.5f) / 50 / 0.01f + Ia_OFFSET;  // [0,1] -> [-0.5,0.5] -> 0.01Ω 50倍 i=u/r
    float Ib = -(dev_red_adc(motor->ch_ib) - 0.5f) / 50 / 0.01f + Ib_OFFSET; // [0,1] -> [-0.5,0.5] -> 0.01Ω 50倍 i=u/r
    float Ic = -(motor->Ia + motor->Ib);
    motor_set_phrase_current(motor, Ia, Ib, Ic);
    // ########################################################
    //                        位置观测
    // ########################################################
    switch (motor->observer)
    {
    case HFI:
        observer_hfi_update(&motor->hfi, motor, dt);
        break;
    default:
        break;
    }
    // ########################################################
}
void motor_position_observer_update(Motor *motor, float dt)
{
    // ########################################################
    //                        位置观测
    // ########################################################
    switch (motor->observer)
    {
    case FAKE:
        observer_fake_update(&motor->fake, motor, dt);
        break;
    case AS5600:
        observer_as5600_update(&motor->as5600, motor, dt);
        break;
    default:
        break;
    }
    // ########################################################
}
/**
 * 设置dq轴电压
 * 中频任务
 */
void motor_ctontrol_update(Motor *motor, float dt)
{
    if (motor->state != Running)
        return;

    // update sin_theta from observer
    motor_set_theta_omega(motor, motor->theta, motor->omega);

    switch (motor->control)
    {
    case OPEN_LOOP_VOLTAGE_CTRL:
    {
        float Ud_target = 0;
        float Uq_target = motor->target;
        motor_open_loop_voltage_ctrl(motor, Ud_target, Uq_target, dt);
        break;
    }
    case CLOSE_LOOP_CURRENT_CTRL:
    {
        float Id_target = 0;
        float Iq_target = motor->target;
        motor_close_loop_current_ctrl(motor, Id_target, Iq_target, dt);
        break;
    }
    case CLOSE_LOOP_VELOCITY_CTRL:
    {
        float velocity_target = motor->target;
        motor_close_loop_velocity_ctrl(motor, velocity_target, dt);
        break;
    }
    case CLOSE_LOOP_POSITION_CTRL:
    {
        float position_target = motor->target;
        motor_close_loop_position_ctrl(motor, position_target, dt);
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
void motor_driver_update(Motor *motor)
{
    motor_set_dq_voltage(motor, motor->Ud, motor->Uq);
}

void motor_foc_loop(Motor *motor, float dt)
{
    motor_current_observer_update(motor, dt);
    motor_ctontrol_update(motor, dt);
    motor_driver_update(motor);
}

/**
 * 开环电压控制
 */
void motor_open_loop_voltage_ctrl(Motor *motor, float Ud_target, float Uq_target, float dt)
{
    motor->Ud = Ud_target;
    motor->Uq = Uq_target;
}
/**
 * 闭环电压控制
 */
void motor_close_loop_voltage_ctrl(Motor *motor, float Ud_target, float Uq_target, float dt)
{
}
/**
 * 闭环电流控制
 */
void motor_close_loop_current_ctrl(Motor *motor, float Id_target, float Iq_target, float dt)
{
    float Id_error = Id_target - lowpass_filter_update(&motor->id_filter, motor->Id, dt); // 计算d轴电流误差
    float Iq_error = Iq_target - lowpass_filter_update(&motor->iq_filter, motor->Iq, dt); // 计算q轴电流误差
    float Ud_target = pid_controller_update(&motor->pid_id_controller, Id_error, dt);     // 更新d轴电流环pid控制器
    float Uq_target = pid_controller_update(&motor->pid_iq_controller, Iq_error, dt);     // 更新q轴电流环pid控制器
    motor_open_loop_voltage_ctrl(motor, Ud_target, Uq_target, dt);                        // 电压开环控制
}
/**
 * 闭环速度控制
 */
void motor_close_loop_velocity_ctrl(Motor *motor, float velocity_target, float dt)
{
    float velocity_error = velocity_target - lowpass_filter_update(&motor->velocity_filter, motor->omega, dt); // 计算速度误差
    float Iq_target = pid_controller_update(&motor->pid_velocity_controller, velocity_error, dt);              // 更新速度环pid控制器
    motor_close_loop_current_ctrl(motor, 0, Iq_target, dt);                                                    // 电流开环控制
}
/**
 * 闭环位置控制
 */
void motor_close_loop_position_ctrl(Motor *motor, float position_target, float dt)
{
    float position_error = position_target - lowpass_filter_update(&motor->position_filter, motor->theta, dt); // 计算位置误差
    float velocity_target = pid_controller_update(&motor->pid_position_controller, position_error, dt);        // 更新位置环pid控制器
    motor_close_loop_velocity_ctrl(motor, velocity_target, dt);                                                // 电流开环控制
}
void motor_set_state(Motor *motor, MotorState state)
{
    motor->state = state;
}
void motor_set_target(Motor *motor, float target)
{
    motor->target = target;
}
void motor_set_control(Motor *motor, ControlType control)
{
    motor->control = control;
}
void motor_set_modulation(Motor *motor, ModulationType modulation)
{
    motor->modulation = modulation;
}