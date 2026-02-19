#ifndef __sensor_as5600_H__
#define __sensor_as5600_H__

#include <stdint.h>
#include "foc_math.h"
#include "pid.h"
#include "pll.h"

typedef struct Motor Motor;

#define AS5600_I2C_ADDR 0x36
#define AS5600_REG_ANGLE 0x0E

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float theta_mes, theta_offset;
        float theta_hat, omega_hat;
        PLL pll;
    } Sensor_as5600;

    void sensor_as5600_init(Sensor_as5600 *self);
    void sensor_as5600_align(Sensor_as5600 *self, Motor *motor);
    void sensor_as5600_update(Sensor_as5600 *self, Motor *motor, float dt);
    float sensor_as5600_i2c_read();
#ifdef __cplusplus
}
#endif

#endif