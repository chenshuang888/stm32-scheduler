#include "led.h"
#include "scheduler.h"

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
