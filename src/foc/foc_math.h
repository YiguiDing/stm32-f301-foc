#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include "math.h"
#include "arm_math.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

    float _normalizeAngle(float a);
    float _phrase_diff(float theta_ref, float theta_mes);
    float _atan2(float y, float x);
    bool check_continuous_condition(uint16_t *counter, bool condition, uint16_t n);

    static inline float _constrain(float value, float min, float max)
    {
        return (value < min ? min : (value > max ? max : value));
    }

#define rad(deg) (deg * 365.0f / M_TWOPI)

#ifndef _ARM_MATH_H
    float _sin(float a);
    float _cos(float a);
#else
#define _sin(a) arm_sin_f32(a)
#define _cos(a) arm_cos_f32(a)
#endif
    void foc_math_test();

#ifdef __cplusplus
}
#endif

#endif