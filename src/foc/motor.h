#ifndef __Motor_H__
#define __Motor_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "dev.h"
#include "pll.h"
#include "foc_math.h"
#include "pid.h"
#include "sensor_i240a2.h"
#include "sensor_as5600.h"
#include "observer_smo.h"
#include "observer_hfi.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 状态
    typedef enum
    {
        Init = 0,
        Running = 1,
        Stop = 2,
    } MotorState;

    // 观测器类型
    typedef enum
    {
        SMO = 0,
        HFI = 1,
    } ObservorType;

    // 传感器类型
    typedef enum
    {
        AS5600 = 0,
    } SensorType;

    // 控制模式
    typedef enum
    {
        OPEN_LOOP_VOLFREQ_CTRL = 0,
        OPEN_LOOP_VOLTAGE_CTRL = 1,
        CLOSE_LOOP_CURRENT_CTRL = 2,
        CLOSE_LOOP_VELOCITY_CTRL = 3,
        CLOSE_LOOP_POSITION_CTRL = 4,
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
        float R, L;
        float KV, KVerps; // rpm/v erps/v
        uint8_t pole_pairs;
        float power_supply;
        // ########################################
        float target;              // 目标值
        MotorState state;          // 电机状态
        bool sensorless;           // 是否无感模式
        ObservorType observer;     // 观测器类型(SMO/HFI)
        SensorType sensor;         // 传感器类型(AS5600)
        bool use_vf_startup;       // 是否使用VF启动
        ControlType control;       // 控制模式
        ModulationType modulation; // 调制模式
        // ########################################
        float Id, Iq;
        float Ialpha, Ibeta;
        float Ia, Ib, Ic;
        // ########################################
        float Ud, Uq;
        float Ualpha, Ubeta;
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
        Sensor_I240A2 i240a2;
        Sensor_as5600 as5600;
        // ########################################
        Observer_SMO smo;
        Observer_HFI hfi;
        // ########################################
    } Motor;
    // ######################################################################
    void motor_init(Motor *motor, float R, float L, float KV, uint8_t pole_pairs, float power_supply);
    // ######################################################################
    void motor_set_theta_omega(Motor *motor, float theta, float omega);
    void motor_set_e_theta_omega(Motor *motor, float e_theta, float e_omega);
    // ######################################################################
    void motor_set_dq_voltage(Motor *motor, float Ud, float Uq);
    void motor_set_abc_current(Motor *motor, float Ia, float Ib, float Ic);
    // ######################################################################
    void motor_sensor_update(Motor *motor, float dt);
    void motor_observer_update(Motor *motor, float dt);
    void motor_control_update(Motor *motor, float dt);
    void motor_driver_update(Motor *motor);
    void motor_foc_loop(Motor *motor, float dt);
    // ######################################################################
    void motor_on_adc_update(Motor *motor); // ADC中断调用：处理电流采样和HFI高频响应计算
    void motor_on_pwm_update(Motor *motor); // PWM中断调用：处理HFI注入电压计算、坐标变换、调制
    // ######################################################################
    void motor_open_loop_voltage_freq_ctrl(Motor *self, float vf_target, float dt);
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