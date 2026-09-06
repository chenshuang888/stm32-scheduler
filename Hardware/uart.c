#include "uart.h"
#include "scheduler.h"
#include "scheduler_queue.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 任务栈大小（单位：字）：本模块自定义，不依赖调度器
   Uart_Task 会调用 Uart_Printf → vsnprintf，栈开销主要来自格式化链，
   256 字 = 1 KB 留足余量（可用 scheduler_stack_free() 复核）。
   Uart_Send_Task 只做一次 HAL_UART_Transmit，调用链很浅，128 字 = 512 B 足够。 */
#define UART_RX_STACK_SIZE   256
#define UART_TX_STACK_SIZE   128

/* 日志队列规格：64 B × 8 条 = 512 B，从 port_mem 一次性分配
 *
 * !! UART_LOG_MAX 同时充当 Uart_Printf 内部临时缓冲的大小 !!
 *    两者必须相等：vsnprintf 在 size 字节内必定写入 '\0'，
 *    而队列按 UART_LOG_MAX 字节整体拷贝。二者一致，
 *    才能保证取出时消息必定带终止符 —— 否则 %s 发送会越界读。 */
#define UART_LOG_MAX       64
#define UART_LOG_DEPTH     8

static uint8_t  rx_buffer[128];
static uint16_t rx_index;
static uint32_t rx_ticks;

/* 日志队列句柄。由 Uart_Init() 创建，
   必须在 scheduler_init() 之后 —— 存储区来自 port_mem。 */
static sched_queue_t *g_log_q;

/*---------------------------------------------------------------------------
 * DMA 发送缓冲与门控标志
 *
 * g_dma_buf 必须是静态的：HAL_UART_Transmit_DMA() 是异步的，函数返回后
 * DMA 仍在读这块内存。若改用任务栈上的局部数组，函数返回或任务被抢占后
 * 内容即失效，发出去的会是乱码。
 *
 * 只需要一个缓冲：Uart_Send_Task 是唯一调用 DMA 发送的地方，
 * 且 g_dma_busy 保证同一时刻只有一笔传输在飞。
 * 排队职责已由日志队列承担，这里不必再做环形缓冲。
 *
 * g_dma_busy 在任务（置 1）与中断（清 0）之间共享，故加 volatile。
 *--------------------------------------------------------------------------*/
static uint8_t          g_dma_buf[UART_LOG_MAX];
static volatile uint8_t g_dma_busy;

void Uart_Init(void)
{
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

    /* 日志队列：须在 scheduler_init() 之后（存储区来自 port_mem）、
       scheduler_start() 之前（让发送任务一启动就能取到消息）。
       创建失败时 g_log_q 为 NULL，Uart_Printf 会安全返回，不影响其它任务。 */
    g_log_q = scheduler_queue_create(UART_LOG_MAX, UART_LOG_DEPTH);

    /* 自注册任务：须在 scheduler_init() 之后、scheduler_start() 之前 */
    scheduler_task_create(Uart_Task, "UART",
                          TASK_PRIO_NORMAL, UART_RX_STACK_SIZE);
    scheduler_task_create(Uart_Send_Task, "UART_TX",
                          TASK_PRIO_NORMAL, UART_TX_STACK_SIZE);
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
            Uart_Printf("UART1 Data:%s\r\n", rx_buffer);
            memset(rx_buffer, 0, rx_index);
            rx_index = 0;

            HAL_UART_AbortReceive(&huart1);
            HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
        }

        scheduler_delay(10);
    }
}

/*---------------------------------------------------------------------------
 * Uart_Printf：任务侧的异步打印入口
 *
 * 只做"格式化 + 投递"，不碰串口硬件：
 *   - 不阻塞：调用方耗时仅 vsnprintf，无需等待波特率移位
 *     （一条 40 字符的日志在 115200 下原本要死等约 3.5 ms）
 *   - 不重入：真正驱动 huart1 的只有 Uart_Send_Task
 *
 * 局部缓冲 tmp 与队列项同为 UART_LOG_MAX 字节，vsnprintf 保证写入 '\0'，
 * 因此入队的定长块必定带终止符，发送侧可安全按字符串处理。
 *--------------------------------------------------------------------------*/
void Uart_Printf(const char *fmt, ...)
{
    char    tmp[UART_LOG_MAX];
    va_list ap;
    int     n;

    if (g_log_q == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    n = vsnprintf(tmp, (size_t)UART_LOG_MAX, fmt, ap);
    va_end(ap);

    if (n < 0)
        return;                             /* 格式化失败 */

    /* n >= UART_LOG_MAX 表示被截断，vsnprintf 已保证 '\0' 结尾，可直接入队 */
    scheduler_queue_send(g_log_q, tmp);     /* 队列满则丢弃，不阻塞调用方 */
}

/*---------------------------------------------------------------------------
 * DMA 发送完成回调
 *
 * 由中断上下文调用（链路见 usart.c 注释），只清标志，不做其它事 ——
 * 绝不能在此调用 scheduler_delay() 等任何可能阻塞的接口。
 *--------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        g_dma_busy = 0;
}

/*---------------------------------------------------------------------------
 * DMA2_Stream7 中断入口
 *
 * 定义在 uart.c 而非 stm32f4xx_it.c，让 DMA 相关代码集中一处；
 * 与 port_timer.c 自行定义 TIM6_DAC_IRQHandler 的做法一致。
 *--------------------------------------------------------------------------*/
void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/*---------------------------------------------------------------------------
 * Uart_Send_Task：串口输出的唯一出口
 *
 * 取一条消息 → 拷进静态缓冲 → 启动 DMA → 立即让出 CPU。
 *
 * 与原先 HAL_UART_Transmit 的区别不在"更快"（波特率不变，仍需 3.5 ms），
 * 而在等待方式：原来那 3.5 ms 是本任务占着 CPU 轮询标志位，同优先级的
 * ICM 完全拿不到 CPU；现在是 delay 让出，ICM 照常运行。
 *
 * 门控式写法：用 continue 回到循环顶部统一判断，而不是写
 * "while (g_dma_busy) scheduler_delay(1)" 这种内层等待 —— 后者一旦
 * 标志位因异常没被清除就会永久死等。
 *--------------------------------------------------------------------------*/
void Uart_Send_Task(void)
{
    char     msg[UART_LOG_MAX];             /* 必须与队列项等宽 */
    uint16_t len;

    while (1)
    {
        if (g_dma_busy)                     /* 上一笔在飞：让出 CPU */
        {
            scheduler_delay(1);
            continue;
        }

        if (!scheduler_queue_recv(g_log_q, msg))
        {
            scheduler_delay(5);             /* 队列空，不是忙等 */
            continue;
        }

        len = (uint16_t)strlen(msg);
        if (len == 0)
            continue;

        memcpy(g_dma_buf, msg, len);

        /* 必须先置位再启动：顺序颠倒的话，若中断抢在置位之前清了标志，
           任务随后置 1，就永远等不到清位。 */
        g_dma_busy = 1;

        if (HAL_UART_Transmit_DMA(&huart1, g_dma_buf, len) != HAL_OK)
            g_dma_busy = 0;                 /* 启动失败须撤销，否则永久卡在"忙" */
    }
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
