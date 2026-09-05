#include "uart.h"
#include "scheduler.h"

/* 任务栈大小（单位：字）：本模块自定义，不依赖调度器
   Uart_Task 栈深主要为 printf 链（链接器实测 ~40 B 为基础），
   256 字 = 1 KB 提供充分余量。 */
#define UART_STACK_SIZE   256

static uint8_t  rx_buffer[128];
static uint16_t rx_index;
static uint32_t rx_ticks;

void Uart_Init(void)
{
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

    /* 自注册任务：须在 scheduler_init() 之后、scheduler_start() 之前 */
    scheduler_task_create(Uart_Task, "UART",
                          TASK_PRIO_NORMAL, UART_STACK_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        rx_ticks = HAL_GetTick();
        if (rx_index < sizeof(rx_buffer) - 1)
            rx_index++;
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

void Uart_Task(void)
{
    while (1)
    {
        if (rx_index > 0 && HAL_GetTick() - rx_ticks > UART_TIMEOUT_MS)
        {
            rx_buffer[rx_index] = '\0';
            printf("UART1 Data:%s\r\n", rx_buffer);
			Led_Init();
            memset(rx_buffer, 0, rx_index);
            rx_index = 0;

            HAL_UART_AbortReceive(&huart1);
            HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
        }

        scheduler_delay(10);
    }
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
