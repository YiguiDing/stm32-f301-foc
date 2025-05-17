#ifndef __OBSERVER_FAKE_H__
#define __OBSERVER_FAKE_H__

typedef struct Motor Motor;

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        float velocity;
        float theta_hat, omega_hat;
    } Observer_Fake;
    void observer_fake_init(Observer_Fake *observer, float velocity);
    void observer_fake_update(Observer_Fake *self, Motor *motor, float dt);
#ifdef __cplusplus
}
#endif

#endif