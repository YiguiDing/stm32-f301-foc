#include "stm32f30x.h"
/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
/* dev includes. */
#include "dev.h"
#include "motor.h"

Motor motor;

void adc_on_update(float dt)
{
    motor_current_observer_update(&motor, dt);
}

TaskHandle_t position_TH = NULL;
static void position_task(void *parameters)
{
    // 1万转/分钟 -> 166转/秒 -> 1.166k电角度转/秒
    float freq = 1.166e3; // 1.166khz
    float dt = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    for (;;)
    {
        motor_position_observer_update(&motor, dt);
        vTaskDelay(ticks);
    }
}

TaskHandle_t control_TH = NULL;
static void control_task(void *parameters)
{
    float freq = 20e3; // 10khz
    float dt = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    for (;;)
    {
        motor_ctontrol_update(&motor, dt);
        vTaskDelay(ticks);
    }
}

void pwm_on_update(float dt)
{
    motor_driver_update(&motor);
}

TaskHandle_t serial_TH = NULL;

typedef struct
{
    float data[12];
    uint32_t tail;
} JustFloatFrame;

static void serial_task(void *parameters)
{
    float freq = 1e3; // 1khz
    float dt = 1 / freq;
    float ticks = configTICK_RATE_HZ / freq;
    JustFloatFrame frame;
    for (;;)
    {
        uint8_t idx = 0;
        frame.data[idx++] = (motor.as5600.theta_hat);
        frame.data[idx++] = (motor.as5600.omega_hat);

        frame.data[idx++] = motor.iq_filter.x_prev;
        frame.data[idx++] = motor.id_filter.x_prev;
        frame.data[idx++] = motor.velocity_filter.x_prev;
        frame.data[idx++] = motor.position_filter.x_prev;

        frame.data[idx++] = motor.Ia;
        frame.data[idx++] = motor.Ib;
        frame.data[idx++] = motor.Ic;

        frame.data[idx++] = motor.Ialpha;
        frame.data[idx++] = motor.Ibeta;

        frame.data[idx++] = motor.Id;
        frame.data[idx++] = motor.Iq;

        // frame.data[2] = motor.Ua;
        // frame.data[3] = motor.Ub;
        // frame.data[4] = motor.Uc;
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
    case 0x10:
        motor.hfi.pll.filter.Ts = val;
        break;
    case 0x11:
        motor.hfi.pll.pid.Kp = val;
        break;
    case 0x12:
        motor.hfi.pll.pid.Ki = val;
        break;
    case 0x21:
        motor.fake.velocity = val;
        break;
    case 0x30:
        motor.as5600.pll.filter.Ts = val;
        break;
    case 0x31:
        motor.as5600.pll.pid.Kp = val;
        break;
    case 0x32:
        motor.as5600.pll.pid.Ki = val;
        break;
    default:
        break;
    }
}

int main(void)
{
    // RTOS文档建议将所有优先级位都指定为抢占优先级位， 不保留任何优先级位作为子优先级位。
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    dev_init();

    motor_init(&motor, 7, 12, 0, 1, 2);

    xTaskCreate(control_task, "control_task", 200, NULL, 2, &control_TH);
    xTaskCreate(position_task, "position_task", 300, NULL, 3, &position_TH);
    xTaskCreate(serial_task, "serial_task", 300, NULL, 4, &serial_TH);

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