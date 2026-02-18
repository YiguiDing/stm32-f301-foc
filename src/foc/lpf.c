
#include "lpf.h"

void lpf_init(LowpassFilter *self, float Ts)
{
    self->Ts = Ts;
}
float lpf_update(LowpassFilter *self, float x_now, float dt)
{
    // dt=>Ts k=>(1/2) 均值滤波 x=> [0.5 * x_prev + 0.5 * x_now]
    // dt=>0 k=>0 维持原值 x=> [1 * x_prev + 0 * x_now]
    float k = dt / (dt + self->Ts);
    float x = self->x_prev + k * (x_now - self->x_prev);
    self->x_prev = x;
    return x;
}
