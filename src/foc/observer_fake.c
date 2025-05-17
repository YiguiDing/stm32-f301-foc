#include "observer_fake.h"
#include "motor.h"
void observer_fake_init(Observer_Fake *observer, float velocity)
{
    observer->velocity = velocity;
    observer->theta_hat = 0;
    observer->omega_hat = 0;
}

void observer_fake_update(Observer_Fake *self, Motor *motor, float dt)
{
    self->theta_hat += self->velocity * dt;
    self->omega_hat = self->velocity;

    motor->theta = self->theta_hat;
    motor->omega = self->omega_hat;
}