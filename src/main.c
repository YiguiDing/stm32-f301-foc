#include "stm32f30x.h"
/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
/* dev includes. */
#include "timer.h"
#include "serial.h"
#include "led.h"

TaskHandle_t task1Handler = NULL;
static void task1(void *parameters)
{
    /* Unused parameters. */
    (void)parameters;
    timer_init();
    serial_init();
    for (;;)
    {
        serial_printf("task1:dt: %d us\r\n", timer_get_dt_us()); // 1000060us -> ≈ 1Hz
        vTaskDelay(configTICK_RATE_HZ / 1);                      // 1hz
    }
}

#define TASK2_STACK_SIZE 100
TaskHandle_t task2Handle;
static StaticTask_t task2TCB;
static StackType_t task2Stack[TASK2_STACK_SIZE];
static void task2(void *parameters)
{
    /* Unused parameters. */
    (void)parameters;
    led_init();
    for (;;)
    {
        led_toggle();
        vTaskDelay(configTICK_RATE_HZ / 50); // 50hz
    }
}
int main(void)
{
    // RTOS文档建议将所有优先级位都指定为抢占优先级位， 不保留任何优先级位作为子优先级位。
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    // 动态分配栈
    xTaskCreate(task1, "task1", 200, NULL, 1, &task1Handler);
    // 静态分配栈
    task2Handle = xTaskCreateStatic(task2,            // code
                                    "task2",          // name
                                    TASK2_STACK_SIZE, // stack size
                                    NULL,             // parameters
                                    2,                // priority
                                    task2Stack,       // stack
                                    &(task2TCB)       // TCB
    );
    /* Start the scheduler. */
    vTaskStartScheduler();
    for (;;)
    {
        /* Should not reach here. */
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