#ifndef _LOWPASS_FILTER_H_
#define _LOWPASS_FILTER_H_

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float Tf;
        float x_prev;
    } LowpassFilter;

    void lpf_init(LowpassFilter *self, float Tf);
    float lpf_update(LowpassFilter *self, float x, float dt);

#ifdef __cplusplus
}
#endif

#endif