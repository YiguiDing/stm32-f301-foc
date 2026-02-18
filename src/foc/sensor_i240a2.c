#include "sensor_I240A2.h"
#include "dev.h"
#include "motor.h"

void sensor_i240a2_init(Sensor_I240A2 *self)
{
    self->Ia_mes = 0;
    self->Ib_mes = 0;
    self->Ic_mes = 0;
    self->Ia_offset = 0;
    self->Ib_offset = 0;
    self->Ic_offset = 0;
}
void sensor_i240a2_align(Sensor_I240A2 *self, Motor *motor)
{
    motor_set_dq_voltage(motor, 0, 0, 0); // 设置DQ电压为0
    dev_delay(50);                        // 延时50ms
    float Ia_avg = 0, Ib_avg = 0, Ic_avg = 0;
    self->Ia_offset = 0;
    self->Ib_offset = 0;
    self->Ic_offset = 0;
    for (uint8_t n = 1; n <= 100; n++)
    {
        // 递归求均值
        // X[n] = X[n-1] + 1/n * (Z[n] - X[n-1])
        Ia_avg += (self->Ia_mes - Ia_avg) / n;
        Ib_avg += (self->Ib_mes - Ib_avg) / n;
        Ic_avg += (self->Ic_mes - Ic_avg) / n;
        dev_delay(10);
    }
    self->Ia_offset = -Ia_avg;
    self->Ib_offset = -Ib_avg;
    self->Ic_offset = -Ic_avg;
}
void sensor_i240a2_update(Sensor_I240A2 *self)
{
    self->Ia_hat = self->Ia_mes + self->Ia_offset;
    self->Ib_hat = self->Ib_mes + self->Ib_offset;
    self->Ic_hat = self->Ic_mes + self->Ic_offset;
}