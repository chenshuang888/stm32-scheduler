#include "uart.h"
#include "scheduler.h"

static uint8_t  rx_buffer[128];
static uint16_t rx_index;
static uint32_t rx_ticks;

void Uart_Init(void)
{
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
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
