#include "stm32f30x.h"
/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
/* dev includes. */
#include "dev.h"
#include "motor.h"
#include "math.h"

Motor motor;

void adc_update()
{
    // static uint8_t cnt = 0;
    // cnt++;
    // if (cnt % 2 == 0)// 只有半个周期能采样到电流，最后发现采样运放没有供电
    //     return;

    // I240A2 电流采样
    // [0,1] -> [-0.5,0.5] /3.3 -> U_r -> 采样电阻0.01Ω 放大倍数50倍 i=u/r
    motor.i240a2.Ia_mes = (ADC_GET_VALUE(0) - 0.5f) / 3.3f / 0.01f / 50;
    motor.i240a2.Ib_mes = -(ADC_GET_VALUE(1) - 0.5f) / 3.3f / 0.01f / 50;
    motor.i240a2.Ic_mes = -(motor.i240a2.Ia_mes + motor.i240a2.Ib_mes);
    // AS5600 analog引脚
    // motor.as5600.theta_mes = ADC_GET_VALUE(2) * M_TWOPI;
}

TaskHandle_t TH_i2c = NULL;
static void i2c_update(void *parameters)
{
    float freq = 1e3; // 1khz
    float Ts = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    for (;;)
    {
        motor.as5600.theta_mes = sensor_as5600_i2c_read();
        vTaskDelay(ticks);
    }
}

void driver_update(float Ts)
{
    pwm1_set_duty(motor.Ta, motor.Tb, motor.Tc);
}

TaskHandle_t TH_observer = NULL;
static void observer_update(void *parameters)
{
    // 1万转/分钟 -> 166转/秒 -> 1.166k电角度转/秒
    float freq = 10e3; // 5khz
    float Ts = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    for (;;)
    {
        motor_sensor_update(&motor, Ts);
        motor_observer_update(&motor, Ts);
        vTaskDelay(ticks);
    }
}

TaskHandle_t TH_control = NULL;
static void control_update(void *parameters)
{
    float freq = 5e3; // 10khz
    float Ts = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    for (;;)
    {
        motor_control_update(&motor, Ts);
        vTaskDelay(ticks);
    }
}
TaskHandle_t TH_swo = NULL;
static void swo_task(void *parameters)
{
    float freq = 0.5e3; // 1khz
    float ticks = configTICK_RATE_HZ / freq;
    uint8_t cnt = 0;
    for (;;)
    {
        swo_printf("%f\n", 123.0f);
        vTaskDelay(ticks);
    }
}

TaskHandle_t TH_serial = NULL;

typedef struct
{
    float data[23];
    uint32_t tail;
} JustFloatFrame;

static void serial_task(void *parameters)
{
    float freq = 1e3; // 1khz
    float ticks = configTICK_RATE_HZ / freq;
    JustFloatFrame frame;
    for (;;)
    {
        uint8_t idx = 0;
        // 电压
        frame.data[idx++] = motor.Ud;
        frame.data[idx++] = motor.Uq;
        frame.data[idx++] = motor.Ualpha;
        frame.data[idx++] = motor.Ubeta;
        frame.data[idx++] = motor.Ua;
        frame.data[idx++] = motor.Ub;
        frame.data[idx++] = motor.Uc;
        // 电流
        frame.data[idx++] = motor.Id;
        frame.data[idx++] = motor.Iq;
        frame.data[idx++] = motor.Ialpha;
        frame.data[idx++] = motor.Ibeta;
        frame.data[idx++] = motor.Ia;
        frame.data[idx++] = motor.Ib;
        frame.data[idx++] = motor.Ic;
        // 速度和位置
        frame.data[idx++] = motor.e_theta;
        frame.data[idx++] = motor.e_omega;
        // 速度和位置
        frame.data[idx++] = motor.smo.theta_raw; // 原始角度
        frame.data[idx++] = motor.smo.theta_hat;
        frame.data[idx++] = motor.smo.omega_hat;
        // 反电动势
        frame.data[idx++] = motor.smo.Ealpha_hat;
        frame.data[idx++] = motor.smo.Ebeta_hat;
        // AS5600传感器数据
        frame.data[idx++] = motor.as5600.theta_hat;
        frame.data[idx++] = motor.as5600.omega_hat;

        // frame.data[idx++] = motor.iq_filter.x_prev;
        // frame.data[idx++] = motor.id_filter.x_prev;
        // frame.data[idx++] = motor.velocity_filter.x_prev;
        // frame.data[idx++] = motor.position_filter.x_prev;

        // frame.data[5] = motor.Ia;
        // frame.data[6] = motor.Ib;
        // frame.data[7] = motor.Ic;

        // frame.data[8] = motor.hfi.i_alpha_avg_0;
        // frame.data[9] = motor.hfi.i_alpha_avg_0;
        // frame.data[10] = motor.hfi.i_alpha_avg_1;
        // frame.data[11] = motor.hfi.i_alpha_avg_1;

        frame.tail = 0x7f800000;
        serial_send((uint8_t *)&frame, sizeof(JustFloatFrame));
        vTaskDelay(ticks);
    }
}

typedef union
{
    uint8_t raw[128];
    struct __attribute__((packed))
    {
        uint8_t hadder;
        uint8_t command;
        float value;
        uint8_t tail;
    } content;
} DataFrame;

void serial_on_receive(uint8_t *data, uint16_t len)
{
    DataFrame *frame = (DataFrame *)data;

    if (!(frame->content.hadder == 0x55 && frame->content.tail == 0xAA))
        return;
    float val = frame->content.value;

    switch (frame->content.command)
    {
    case 0x01:
        motor.target = val;
        break;
    case 0x02:
        motor.control = (ControlType)val;
        break;
    }
}

int main(void)
{
    // RTOS文档建议将所有优先级位都指定为抢占优先级位， 不保留任何优先级位作为子优先级位。
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    // swo_init(0xFFFFFFFF, 72000000, 2000000);
    // led_init();
    //
    adc_init();
    adc_set_callback(adc_update);
    //
    pwm1_init();
    pwm1_set_freq(36e3); // 24khz
    pwm1_set_callback(driver_update);
    pwm1_enable();
    //
    i2c_init();
    timer_init();
    serial_init();

    
    // motor_init(&motor, 8.25, 4.25e-3, 110, 7, 12); // 2208电机 kv=110 pp=7 R=8Ω L=4.25mH Vmax=12V
    motor_init(&motor, 112e-3, 9e-6, 2300, 7, 12); // 2204电机 kv=2300 pp=7 R=112mΩ L=9uH Vmax=12V Imax=12A Pmax=140w
    // motor_init(&motor, 100e-3, 42.3e-6, 650, 10, 12); // 3505电机 kv=650 pp=10 R=100mΩ L=42.3uH Vmax=12~16V Imax=17A Pmax=240w
    xTaskCreate(i2c_update, "i2c_update", 100, NULL, 2, &TH_i2c);
    xTaskCreate(control_update, "control_update", 200, NULL, 2, &TH_control);
    xTaskCreate(observer_update, "observer_update", 100, NULL, 3, &TH_observer);
    xTaskCreate(serial_task, "serial_task", 100, NULL, 4, &TH_serial);
    // xTaskCreate(swo_task, "swo_task", 500, NULL, 4, &TH_swo);
    motor_set_target(&motor, 0);

    vTaskStartScheduler();
    for (;;)
    {
    }
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW > 0)

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* Check pcTaskName for the name of the offending task,
     * or pxCurrentTCB if pcTaskName has itself been corrupted. */
    (void)xTask;
    (void)pcTaskName;
    serial_printf("Stack overflow in task: %s\r\n", pcTaskName);
    for (;;)
        ;
}

#endif

#if (configUSE_MALLOC_FAILED_HOOK > 0)

void vApplicationMallocFailedHook(void)
{
    serial_printf("Malloc failed!\r\n");
    for (;;)
        ;
}
#endif