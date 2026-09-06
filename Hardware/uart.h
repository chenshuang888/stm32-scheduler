#ifndef __UART_H
#define __UART_H

#include "main.h"

#define UART_TIMEOUT_MS  10

void Uart_Init(void);
void Uart_Task(void);        /* 接收任务：收帧 + 投递回显消息 */
void Uart_Send_Task(void);   /* 发送任务：消费日志队列，是串口输出的唯一出口 */

/*---------------------------------------------------------------------------
 * 异步打印：格式化后投递进日志队列，由 Uart_Send_Task 统一发送
 *
 * !! 调度器启动后，所有任务一律用本函数，不要直接 printf !!
 *    直接 printf 会绕过队列、与 Uart_Send_Task 争抢 huart1，
 *    表现为 HAL_UART_Transmit 卡住等 gState，偶发延迟且极难定位。
 *    调度器启动前（main / 各 Init 内）仍可直接用 printf。
 *
 * 队列满时丢弃本条消息，不阻塞调用方。
 * 单条消息超过 UART_LOG_MAX-1 字符会被截断。
 *--------------------------------------------------------------------------*/
void Uart_Printf(const char *fmt, ...);

#endif
