#include "foc_math.h"
#include "stdint.h"

float _normalizeAngle(float a)
{
    a = fmodf(a, M_TWOPI);
    if (a < 0)
        a += M_TWOPI;
    return a;
}

float _phrase_diff(float theta_ref, float theta_mes)
{
    float diff = _normalizeAngle(theta_ref) - _normalizeAngle(theta_mes);
    if (diff > M_PI)
        diff -= M_TWOPI;
    else if (diff < -M_PI)
        diff += M_TWOPI;
    return diff; // diff ∈ [-M_PI,+M_PI]
}

// fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
// Via Odrive project
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
// The origin for Odrive atan2 is public domain. Thanks to Odrive for making
// it easy to borrow.
float _atan2(float y, float x)
{
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    float a = fmin(abs_x, abs_y) / (fmax(abs_x, abs_y));
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    if (x < 0.0f)
        r = 3.14159274f - r;
    if (y < 0.0f)
        r = -r;
    return r;
}

#ifndef _ARM_MATH_H
/**
 * @param a float [0,2PI]
 * @return float [-1,1]
 */
float _sin(float a)
{
    static uint16_t sine_array[65] = {
        //  for(float a=0;a<=1;a+=M_PI/65) sin(a);
        0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
        6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
        12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
        18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
        23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
        27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
        30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
        32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767 //
    };
    int32_t t1, t2;
    uint16_t idx = a / M_TWOPI * 0xffff; // [0,2PI] => [0,0xffff]
    uint16_t frac = idx & 0xff;
    idx = (idx >> 8) & 0xff;
    if (idx < 64)
    {
        t1 = sine_array[idx];
        t2 = sine_array[idx + 1];
    }
    else if (idx < 128)
    {
        t1 = sine_array[128 - idx];
        t2 = sine_array[128 - (idx + 1)];
    }
    else if (idx < 192)
    {
        t1 = -(int32_t)sine_array[-128 + idx];
        t2 = -(int32_t)sine_array[-128 + (idx + 1)];
    }
    else
    {
        t1 = -(int32_t)sine_array[256 - idx];
        t2 = -(int32_t)sine_array[256 - (idx + 1)];
    }
    return (t1 + (((t2 - t1) * frac / 256.0f))) / 32768.0f;
}

/**
 * @param a float [0,2PI]
 * @return float [-1,1]
 */
float _cos(float a)
{
    // _cos和sin相位差90°，所以加上PI/2
    return _sin(_normalizeAngle(a + M_PI_2));
}
#else
// _sin -> arm_sin_f32
// _cos -> arm_cos_f32
#endif

/**
 * 检测是否连续n次满足条件
 * @param counter 计数器指针
 * @param condition 当前是否满足条件
 * @param n 连续满足的次数阈值
 * @return true 如果连续满足n次, false 否则
 */
bool check_continuous_condition(uint16_t *counter, bool condition, uint16_t n)
{
    if (condition)
    {
        if ((*counter)++ >= n)
        {
            *counter = 0;
            return true;
        }
        return false;
    }
    *counter = 0;
    return false;
}

#include "dev.h"
void foc_math_test()
{
    serial_init();
    timer_init();
    float theta = 0;
    uint8_t cnt = 0;
    while (1)
    {
        float sin_theta = _sin(theta);
        float cos_theta = _cos(theta);
        theta += 0.01;
        if (theta > M_TWOPI)
            theta = 0;
        uint32_t dt_us = timer_get_dt_us();

        cnt++;
        if (cnt == 0)
            serial_printf("%ld\n", dt_us);
    }
}