#ifndef __UART_H
#define __UART_H

#include "main.h"

#define UART_TIMEOUT_MS  10

void Uart_Init(void);
void Uart_Task(void);

#endif
