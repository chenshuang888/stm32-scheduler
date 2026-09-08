#include "uart.h"
#include "scheduler.h"
#include "scheduler_queue.h"
#include "scheduler_sem.h"
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

/* 串口接收配置
 *
 * UART_RX_BUF_SIZE：DMA 循环缓冲大小。硬件不停往里填、到尾绕回，
 *   软件只管按读指针取走。取不及时最老的数据会被覆盖，故留 256 B。
 * UART_RX_SEM_MAX：信号量上限。IDLE 中断只 give 不累加到无穷，
 *   设 8 表示"最多记住 8 次未处理的帧到达事件"，超出则合并。 */
#define UART_RX_BUF_SIZE   256
#define UART_RX_SEM_MAX    8

/*---------------------------------------------------------------------------
 * 串口接收：DMA 循环缓冲 + 读指针 + 接收信号量
 *
 * g_rx_buf 由 DMA 硬件独自写入（无需 CPU 参与），g_rx_read 是软件侧的
 * 读指针。硬件写指针不记录在变量里，而是每次从 DMA 的 NDTR 寄存器推算：
 *
 *     NDTR   = 本次传输"还剩多少字节没传"
 *     write  = BUF_SIZE - NDTR   （即已写入的总字节数对 BUF_SIZE 取模）
 *
 * 循环模式下 NDTR 递减到 0 后由硬件自动重载为 BUF_SIZE，周而复始。
 *
 * 数据流的并发只有"DMA 硬件写 vs 任务读"，没有两个 CPU 执行流同时改
 * 同一个变量，所以这里不需要临界区 —— 只需保证读到的是一致的 NDTR 即可
 * （NDTR 是外设寄存器，单次访问原子）。
 *--------------------------------------------------------------------------*/
static uint8_t      g_rx_buf[UART_RX_BUF_SIZE];
static uint16_t     g_rx_read;
static sched_sem_t *g_rx_sem;

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
    /* 日志队列：须在 scheduler_init() 之后（存储区来自 port_mem）、
       scheduler_start() 之前（让发送任务一启动就能取到消息）。
       创建失败时 g_log_q 为 NULL，Uart_Printf 会安全返回，不影响其它任务。 */
    g_log_q = scheduler_queue_create(UART_LOG_MAX, UART_LOG_DEPTH);

    /* 接收信号量：初值 0（还没有数据），上限 UART_RX_SEM_MAX */
    g_rx_sem = scheduler_sem_create(0U, UART_RX_SEM_MAX);

    /* 启动 DMA 循环接收：此后硬件自动把收到的字节填进 g_rx_buf，
       到尾自动绕回，全程不需要 CPU 参与。 */
    HAL_UART_Receive_DMA(&huart1, g_rx_buf, UART_RX_BUF_SIZE);

    /* 使能 IDLE 中断：一帧收完后线路空闲一个帧时间即触发。
       这是循环 DMA 下判断"一帧结束"的唯一手段 —— 循环模式的 DMA
       永远不会产生"传输完成"中断。 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 自注册任务：须在 scheduler_init() 之后、scheduler_start() 之前 */
    scheduler_task_create(Uart_Task, "UART",
                          TASK_PRIO_NORMAL, UART_RX_STACK_SIZE);
    scheduler_task_create(Uart_Send_Task, "UART_TX",
                          TASK_PRIO_NORMAL, UART_TX_STACK_SIZE);
}

/*---------------------------------------------------------------------------
 * IDLE 中断处理：一帧接收完毕的信号
 *
 * 触发条件（务必理解准确，这是本模块的核心）：
 *   **接收到数据之后**，RX 线路保持高电平（无数据）达一个完整帧的时间。
 *   —— 不是"线路一直空闲就触发"。上电后从未收到过数据时，
 *      IDLE 不会置位，不会反复进中断。
 *
 * 115200 下一个完整帧约 10 bit ≈ 87 µs，即一帧数据收完后约 87 µs 触发，
 * 因此它能准确标记"这一帧结束了，后面没跟着数据"，是不定长帧的标准判据。
 *
 * !! 必须清除标志 !!
 *    IDLE 标志不会自动清零，不清会反复进入本中断把系统拖死。
 *    F4 的清除序列是"先读 SR 再读 DR"，HAL 封装为
 *    __HAL_UART_CLEAR_IDLEFLAG()。
 *
 * 本函数运行在中断上下文，只做"清标志 + 释放信号量"，
 * 绝不在此做格式化、拷贝等耗时操作。
 *--------------------------------------------------------------------------*/
void Uart_IdleIrqHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == RESET)
        return;

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    scheduler_sem_give(g_rx_sem);       /* 唤醒接收任务 */
}

/*---------------------------------------------------------------------------
 * 从 DMA 循环缓冲取出自上次读取以来的全部新字节
 *
 * 返回取出的字节数（0 表示无新数据）。
 * 缓冲可能绕回，故最多分两段拷贝，最终在 dst 中得到一段连续数据。
 *--------------------------------------------------------------------------*/
static uint16_t uart_rx_pull(uint8_t *dst)
{
    uint16_t ndtr;
    uint16_t write;
    uint16_t len;
    uint16_t first;

    /* NDTR = 本次传输还剩多少字节未传 → 已写入字节数（取模即写指针） */
    ndtr  = (uint16_t)(hdma_usart1_rx.Instance->NDTR);
    write = (uint16_t)((UART_RX_BUF_SIZE - ndtr) % UART_RX_BUF_SIZE);

    if (write == g_rx_read)
        return 0;                                   /* 没有新数据 */

    len = (write > g_rx_read) ? (uint16_t)(write - g_rx_read)
                              : (uint16_t)(UART_RX_BUF_SIZE - g_rx_read + write);

    /* 分两段拷出，拼成一段连续数据 */
    first = (uint16_t)(UART_RX_BUF_SIZE - g_rx_read);
    if (first > len)
        first = len;
    memcpy(dst, &g_rx_buf[g_rx_read], first);
    if (len > first)
        memcpy(&dst[first], &g_rx_buf[0], len - first);

    g_rx_read = write;

    return len;
}

/*---------------------------------------------------------------------------
 * Uart_Task：接收任务（事件驱动）
 *
 * 阻塞等信号量 → 被 IDLE 中断唤醒 → 从 DMA 缓冲取走这一帧 → 投递回显。
 * 没有数据时任务完全不参与调度，CPU 全给 Idle —— 相比原先 10 ms 轮询
 * 单字节接收，既省掉了轮询，也不再需要重启接收。
 *--------------------------------------------------------------------------*/
void Uart_Task(void)
{
    char     frame[UART_RX_BUF_SIZE + 1];           /* +1 留给终止符 */
    uint16_t len;

    while (1)
    {
        if (!scheduler_sem_take(g_rx_sem, 0))       /* 0 = 永久等待 */
        {
            /* 正常不会走到这里（仅当 g_rx_sem 创建失败）。
               必须延时，否则退化成死循环会把同优先级任务饿死。 */
            scheduler_delay(10);
            continue;
        }

        len = uart_rx_pull((uint8_t *)frame);
        if (len == 0)
            continue;

        frame[len] = '\0';
        Uart_Printf("UART1 Data:%s\r\n", frame);
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
 * 队列侧用 recv_block 永久等待：没消息时本任务完全不参与调度，
 * 由生产者 send 时直接唤醒 —— 既没有 5 ms 轮询唤醒的开销，
 * 响应延迟也从"最多一个轮询周期"降到微秒级。
 *
 * DMA 侧仍用 delay(1) 轮询（可选项，未做阻塞化）：传输只需几毫秒，
 * 轮询代价很小，先保持简单。
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

        if (!scheduler_queue_recv_block(g_log_q, msg, 0))   /* 0 = 永久等待 */
        {
            /* 永久等待正常情况下不会返回 0 —— 只有队列创建失败
               （g_log_q == NULL）时才会走到这里。这里必须延时，
               否则会退化成不打折扣的死循环，而同优先级不抢占，
               其它任务将被彻底饿死。 */
            scheduler_delay(10);
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
