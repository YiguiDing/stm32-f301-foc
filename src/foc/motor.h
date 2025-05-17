#ifndef __Motor_H__
#define __Motor_H__

#include <stdint.h>
#include <math.h>
#include "dev.h"
#include "pll.h"
#include "foc_math.h"
#include "pid_controller.h"
#include "observer_fake.h"
#include "observer_as5600.h"
#include "observer_hfi.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 状态
    typedef enum
    {
        Align = 0,
        Running = 1,
        Stop = 2,
    } MotorState;

    // 控制模式
    typedef enum
    {
        FAKE = 0,
        AS5600 = 1,
        HFI = 2,
    } ObservorType;

    // 控制模式
    typedef enum
    {
        OPEN_LOOP_VOLTAGE_CTRL = 0,
        CLOSE_LOOP_CURRENT_CTRL = 1,
        CLOSE_LOOP_VELOCITY_CTRL = 2,
        CLOSE_LOOP_POSITION_CTRL = 3,
    } ControlType;

    // 调制模式
    typedef enum
    {
        SPWM = 0,
        SVPWM = 1,
    } ModulationType;

    typedef struct Motor
    {
        // ########################################
        uint8_t pole_pairs;
        float power_supply;
        uint8_t ch_ia, ch_ib, ch_theta;
        // ########################################
        float target;              // 目标值
        MotorState state;          // 电机状态
        ObservorType observer;     // 观测模式
        ControlType control;       // 控制模式
        ModulationType modulation; // 调制模式
        // ########################################
        float Id, Iq;
        float Ia, Ib, Ic;
        float Ialpha, Ibeta;
        // ########################################
        float Ud, Uq;
        float Uahpha, Ubeta;
        float Ua, Ub, Uc;
        float Ta, Tb, Tc;
        // ########################################
        float theta;       // 实际角度
        float omega;       // 实际角速度
        float e_theta;     // 实际电角度
        float e_omega;     // 实际电角速度
        float sin_e_theta; // sin(e_theta)
        float cos_e_theta; // cos(e_theta)
        // ########################################
        LowpassFilter id_filter, iq_filter;
        LowpassFilter velocity_filter;
        LowpassFilter position_filter;
        // ########################################
        float limit_voltage;
        float limit_current;
        float limit_velocity;
        // clsoe_loop_voltage_ctrl
        PidController pid_ud_controller;
        PidController pid_uq_controller;
        // clsoe_loop_current_ctrl
        PidController pid_id_controller;
        PidController pid_iq_controller;
        // clsoe_loop_velocity_ctrl
        PidController pid_velocity_controller;
        // clsoe_loop_position_ctrl
        PidController pid_position_controller;
        // ########################################
        Observer_AS5600 as5600;
        Observer_HFI hfi;
        Observer_Fake fake;
        // ########################################
    } Motor;
    // ######################################################################
    void motor_init(Motor *motor, uint8_t pole_pairs, float power_supply, uint8_t ch_ia, uint8_t ch_ib, uint8_t ch_theta);
    // ######################################################################
    void motor_set_theta_omega(Motor *motor, float theta, float omega);
    void motor_set_e_theta_omega(Motor *motor, float e_theta, float e_omega);
    // ######################################################################
    void motor_set_dq_voltage(Motor *motor, float Ud, float Uq);
    void motor_set_phrase_current(Motor *motor, float Ia, float Ib, float Ic);
    // ######################################################################
    void motor_current_observer_update(Motor *motor, float dt);
    void motor_position_observer_update(Motor *motor, float dt);
    void motor_ctontrol_update(Motor *motor, float dt);
    void motor_driver_update(Motor *motor);
    void motor_foc_loop(Motor *motor, float dt);
    // ######################################################################
    void motor_open_loop_voltage_ctrl(Motor *motor, float Ud_target, float Uq_target, float dt);
    void motor_close_loop_current_ctrl(Motor *motor, float Id_target, float Iq_target, float dt);
    void motor_close_loop_velocity_ctrl(Motor *motor, float velocity_target, float dt);
    void motor_close_loop_position_ctrl(Motor *motor, float position_target, float dt);
    // ######################################################################
    void motor_set_state(Motor *motor, MotorState state);
    void motor_set_target(Motor *motor, float target);
    void motor_set_control(Motor *motor, ControlType control);
    void motor_set_modulation(Motor *motor, ModulationType modulation);
    // ######################################################################
#ifdef __cplusplus
}
#endif

#endif