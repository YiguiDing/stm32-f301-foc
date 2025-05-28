#include "dev.h"

void dev_init()
{
    led_init();
    adc_init();
    pwm_init();
    i2c_init();
    timer_init();
    serial_init();
}

