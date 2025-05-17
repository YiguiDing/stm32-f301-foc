#include "observer_as5600.h"

#include "dev.h"
#include "math.h"
#include "motor.h"

void observer_as5600_init(Observer_AS5600 *self, uint8_t ch)
{
    self->ch = ch;
    self->addr = 0x36;
    self->offset = 0;
    self->theta_mes = self->theta_hat = self->omega_hat = 0;
    pll_init(&self->pll, 0.000001, 1500, 500);
}

float observer_as5600_analog_read(Observer_AS5600 *self)
{
    // adc
    return dev_red_adc(self->ch) * M_TWOPI;
}

float observer_as5600_i2c_read(Observer_AS5600 *self)
{
#define REG_RAW_ANGLE 0x0c
#define REG_ANGLE 0x0E
    uint8_t data[2] = {0};
    uint16_t angle = 0x0000;
    if (!i2c_read(self->addr << 1, REG_ANGLE, data, 2))
        angle = data[0] << 8 | data[1];
    return angle * M_TWOPI / 0x0FFF; // 12bit精度
}

#define observer_as5600_read observer_as5600_i2c_read

void observer_as5600_align(Observer_AS5600 *self, Motor *motor)
{
    motor->state = Align;
    motor_set_theta_omega(motor, 0, 0);                        //
    motor_set_dq_voltage(motor, 0.2 * motor->power_supply, 0); //
    dev_delay(1000);                                           // 1s
    float x = 0;
    for (uint8_t n = 1; n <= 100; n++)
    {
        // 递归求均值
        // X[n] = X[n-1] + 1/n * (Z[n] - X[n-1])
        float z = observer_as5600_read(self);
        x = x + (z - x) / n;
        dev_delay(10);
    }
    self->offset = -x;
    motor_set_dq_voltage(motor, 0, 0);
    motor->state = Running;
}
void observer_as5600_update(Observer_AS5600 *self, Motor *motor, float dt)
{
    // 测量角度
    self->theta_mes = _normalizeAngle(M_TWOPI - (observer_as5600_read(self) + self->offset));
    // self->theta_mes = ((self->theta_mes - 0.35) / (6.25 - 0.35)) * M_TWOPI;  // [0.355,6.258] => [0,2PI]
    // 锁相环更新
    pll_update(&self->pll, self->theta_mes, dt);
    // 角度估计
    self->theta_hat = self->pll.theta;
    self->omega_hat = self->pll.omega;
    //
    motor->theta = self->theta_hat;
    motor->omega = self->omega_hat;
}