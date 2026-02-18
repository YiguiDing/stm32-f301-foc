#include "sensor_as5600.h"

#include "dev.h"
#include "math.h"
#include "motor.h"

void sensor_as5600_init(Sensor_as5600 *self)
{
    self->theta_offset = 0;
    self->theta_mes = 0;
    self->theta_mes = 0;
    self->omega_hat = 0;
    pll_init(&self->pll, 0.000001, 1500, 500, 500);
}

float sensor_as5600_i2c_read()
{
    uint8_t data[2] = {0};
    uint16_t angle = 0x0000;
    if (!i2c_read(AS5600_I2C_ADDR, AS5600_REG_ANGLE, data, 2))
        angle = data[0] << 8 | data[1];
    return angle * M_TWOPI / 0x0FFF; // 12bit精度
}

void sensor_as5600_align(Sensor_as5600 *self, Motor *motor)
{
    motor_set_dq_voltage(motor, 0.2 * motor->power_supply, 0, 0);
    dev_delay(50);
    float avg = 0;
    for (uint8_t n = 1; n <= 100; n++)
    {
        // 递归求均值
        // X[n] = X[n-1] + 1/n * (Z[n] - X[n-1])
        avg += (self->theta_mes - avg) / n;
        dev_delay(10);
    }
    self->theta_offset = -avg;
    motor_set_dq_voltage(motor, 0, 0, 0);
    dev_delay(50);
}
void sensor_as5600_update(Sensor_as5600 *self, Motor *motor, float dt)
{
    // 测量角度
    self->theta_hat = _normalizeAngle(self->theta_mes + self->theta_offset);
    // 锁相环更新
    self->theta_hat = pll_update(&self->pll, self->theta_hat - self->pll.theta_hat, dt);
    self->theta_mes = self->pll.omega_hat;
}