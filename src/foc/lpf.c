
#include "lpf.h"

void lpf_init(LowpassFilter *self, float Tf)
{
    self->Tf = Tf;
}
float lpf_update(LowpassFilter *self, float x_now, float Ts)
{
    float alpha = self->Tf / (self->Tf + Ts);
    float x = alpha * self->x_prev + (1 - alpha) * x_now;
    self->x_prev = x;
    return x;
}
