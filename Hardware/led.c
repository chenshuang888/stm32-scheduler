#include "led.h"
#include "scheduler.h"

/* 任务栈大小（单位：字）：本模块自定义，不依赖调度器
   Led_Task 栈深极浅（两次 HAL_GPIO_WritePin，链接器实测 16 B），
   128 字 = 512 B 已含充足余量。 */
#define LED_STACK_SIZE   128

/*---------------------------------------------------------------------------
 * Led_Init：创建 LED 流水灯任务
 * 须在 scheduler_init() 之后、scheduler_start() 之前调用。
 *-------------------------------------------------------------------------*/
void Led_Init(void)
{
    scheduler_task_create(Led_Task, "LED",
                          TASK_PRIO_NORMAL, LED_STACK_SIZE);
}

void Led_Task(void)
{
    static uint8_t idx = 0;
    uint16_t pins[] = {LED_B_Pin, LED_G_Pin, LED_R_Pin};

    while (1)
    {
        HAL_GPIO_WritePin(LED_B_GPIO_Port,
                          LED_B_Pin | LED_G_Pin | LED_R_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, pins[idx], GPIO_PIN_SET);

        if (++idx >= 3) idx = 0;

        scheduler_delay(500);
    }
}
