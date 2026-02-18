#ifndef __SENSOR_I240A2_H__
#define __SENSOR_I240A2_H__

typedef struct Motor Motor;

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float Ia_mes, Ib_mes, Ic_mes;          // 电流采样值
        float Ia_offset, Ib_offset, Ic_offset; // 电流偏置值
        float Ia_hat, Ib_hat, Ic_hat;          // 电流估计
    } Sensor_I240A2;
    void sensor_i240a2_init(Sensor_I240A2 *self);
    void sensor_i240a2_align(Sensor_I240A2 *self, Motor *motor);
    void sensor_i240a2_update(Sensor_I240A2 *self);

#ifdef __cplusplus
}
#endif

#endif