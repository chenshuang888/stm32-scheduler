#include "scheduler_queue.h"
#include "scheduler.h"
#include "port_mem.h"
#include <string.h>

/*===========================================================================
 * scheduler_queue 实现
 *
 * 数据结构：head/tail/count 三变量环形缓冲（定长项）
 *
 *     ┌───┬───┬───┬───┬───┬───┬───┬───┐
 *     │   │ ▓ │ ▓ │ ▓ │   │   │   │   │    capacity = 8
 *     └───┴───┴───┴───┴───┴───┴───┴───┘
 *           ↑               ↑
 *         head            tail
 *       (下次取)         (下次放)
 *
 *   count == 0         → 空
 *   count == capacity  → 满
 *
 * 用 count 而不是"（tail+1)%cap==head 留一格"的经典判据：
 * 多占一个字节变量，换来空/满判断的一目了然，也免去了 capacity 语义
 * （"实际能装 capacity-1 条"）带来的误解。
 *
 * ---------------------------------------------------------------------------
 * 临界区：一律裸 __disable_irq() / __enable_irq()
 *
 * 之所以不封装成"保存/恢复 PRIMASK"的形式：那种写法是为了支持"临界区里
 * 再调用一个自带临界区的函数"（嵌套）。而嵌套临界区本身就是代码异味，
 * 本模块通过约定彻底避免它 —— 凡是内核提供的 wait_/wake_ 系列函数都不
 * 自己关中断，由调用者统一持锁。于是全工程只有一层临界区，
 * 裸开关就足够，不需要任何包装。
 *
 * 为什么关中断足以保护队列：
 *   head/tail/count 的并发只可能来自"任务 vs 中断"。关中断后同优先级任务
 *   不会抢占、中断也不会插入，读改写序列天然原子。
 *
 * !! 临界区内禁止的事 !!
 *   1. 不做耗时操作（vsnprintf、大块拷贝）—— 关中断太久会丢时基 tick
 *   2. 不调用 scheduler_delay() 等会阻塞的接口
 *   3. 临界区不能跨越阻塞点，否则保护就断了
 *=========================================================================*/

/* 索引推进：不用 % 取模，避免引入除法指令 */
static uint16_t q_next(sched_queue_t *q, uint16_t idx)
{
    uint16_t n = idx + 1U;
    return (n >= q->capacity) ? 0U : n;
}

/*---------------------------------------------------------------------------
 * 创建队列
 *
 * 分配顺序：先大（存储区）后小（控制块）。
 *   理由与 scheduler_task_create() 一致：池子不支持释放，无法回滚。
 *   先分大块，则"大块失败"时不浪费小块；"大块成功、小块失败"只是浪费
 *   一块存储区 —— 反过来（先小后大）若大块失败，小块就白白浪费了。
 *   两者都会泄漏，但先大后小时单次泄漏的期望损失更小。
 *
 * 全程关中断：port_mem 的游标与队列结构体都没有原子保护，
 *   必须避免被时基中断打断后读到半成品状态。
 *-------------------------------------------------------------------------*/
sched_queue_t *scheduler_queue_create(uint16_t item_size, uint16_t capacity)
{
    sched_queue_t *q;
    uint8_t       *buf;
    uint32_t       aligned;
    uint32_t       total;

    if (item_size == 0U || capacity == 0U)
        return NULL;

    /* 每项大小向上对齐到 4 字节。
       存储区首地址由 port_mem 保证 8 对齐，每项再对齐到 4，
       则任意一项的起始地址都保持 4 对齐 —— 免去非对齐访问隐患，
       也让调试时按字观察内存更直观。 */
    aligned = ((uint32_t)item_size + 3U) & ~3UL;
    if (aligned > 0xFFFFUL)
        return NULL;                       /* 65533~65535 对齐后回绕 */

    /* 防乘法溢出：capacity × aligned 必须仍在 uint32 范围内 */
    if ((uint32_t)capacity > (0xFFFFFFFFUL / aligned))
        return NULL;

    total = (uint32_t)capacity * aligned;

    __disable_irq();

    buf = (uint8_t *)scheduler_port_mem_alloc(total);
    if (buf == NULL)
    {
        __enable_irq();
        return NULL;
    }

    q = (sched_queue_t *)scheduler_port_mem_alloc(sizeof(sched_queue_t));
    if (q == NULL)
    {
        __enable_irq();
        return NULL;                       /* 存储区已泄漏，但池本就不支持回收 */
    }

    q->buf       = buf;
    q->item_size = (uint16_t)aligned;
    q->capacity  = capacity;
    q->head      = 0U;
    q->tail      = 0U;
    q->count     = 0U;

    __enable_irq();

    return q;
}

/*---------------------------------------------------------------------------
 * 入队
 *
 * 放入数据后立刻尝试唤醒一个等待者 —— 唤醒动作必须在临界区内完成，
 * 否则"放入"与"唤醒"之间若被 tick 插入，可能把刚唤醒的任务又误判为超时。
 *--------------------------------------------------------------------------*/
int scheduler_queue_send(sched_queue_t *q, const void *item)
{
    uint8_t *dst;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    __disable_irq();

    if (q->count >= q->capacity)           /* 满：丢弃新消息 */
    {
        __enable_irq();
        return 0;
    }

    dst = q->buf + (uint32_t)q->tail * q->item_size;
    memcpy(dst, item, q->item_size);

    q->tail  = q_next(q, q->tail);
    q->count = (uint16_t)(q->count + 1U);

    scheduler_wake_one(q);                 /* 队列自身即"等待对象" */

    __enable_irq();

    return 1;
}

/*---------------------------------------------------------------------------
 * 出队（非阻塞）
 *--------------------------------------------------------------------------*/
int scheduler_queue_recv(sched_queue_t *q, void *item)
{
    const uint8_t *src;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    __disable_irq();

    if (q->count == 0U)                    /* 空：非阻塞，立即返回 */
    {
        __enable_irq();
        return 0;
    }

    src = q->buf + (uint32_t)q->head * q->item_size;
    memcpy(item, src, q->item_size);

    q->head  = q_next(q, q->head);
    q->count = (uint16_t)(q->count - 1U);

    __enable_irq();

    return 1;
}

/*---------------------------------------------------------------------------
 * 出队（阻塞等待）
 *
 *   timeout_ms = 0 → 永久等待，直到有消息入队
 *   timeout_ms > 0 → 限时等待，到期返回 0
 *
 * 返回：1 取到消息；0 超时（或调度器未启动、参数非法）
 *
 * 为什么整体是 for(;;) 循环：
 *   被唤醒只代表"曾经有人往队列里放过东西"，不代表这一条还在 ——
 *   若有多个消费者，可能被别人先取走。所以醒来后必须重新判断，
 *   取不到就继续等。
 *
 * 为什么"判空"与"登记阻塞"必须在同一个临界区内：
 *   见 scheduler_wait_prepare() 的注释（丢失唤醒问题）。
 *   注意下面取消息的代码是内联写的，而不是调用 scheduler_queue_recv()——
 *   就是为了不出现"临界区里再进一次临界区"的嵌套。
 *--------------------------------------------------------------------------*/
int scheduler_queue_recv_block(sched_queue_t *q, void *item, uint32_t timeout_ms)
{
    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    for (;;)
    {
        __disable_irq();

        if (q->count > 0U)
        {
            const uint8_t *src = q->buf + (uint32_t)q->head * q->item_size;
            memcpy(item, src, q->item_size);
            q->head  = q_next(q, q->head);
            q->count = (uint16_t)(q->count - 1U);
            __enable_irq();
            return 1;
        }

        /* 调度器尚未启动：无法阻塞，退化成非阻塞语义 */
        if (current_tcb == NULL)
        {
            __enable_irq();
            return 0;
        }

        scheduler_wait_prepare(q, timeout_ms);
        __enable_irq();

        /* ===== 任务在此挂起；被唤醒后从下一行继续 ===== */
        if (current_tcb->wait_result == 0U)
            return 0;                      /* 超时 */

        /* 被唤醒：回到循环顶部重新取（可能已被别的消费者取走） */
    }
}

/*---------------------------------------------------------------------------
 * 窥视队头（不移除）
 *--------------------------------------------------------------------------*/
int scheduler_queue_peek(sched_queue_t *q, void *item)
{
    const uint8_t *src;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    __disable_irq();

    if (q->count == 0U)
    {
        __enable_irq();
        return 0;
    }

    src = q->buf + (uint32_t)q->head * q->item_size;
    memcpy(item, src, q->item_size);

    __enable_irq();

    return 1;
}

/*---------------------------------------------------------------------------
 * 状态查询
 *
 * count/space 也要关中断读取：它们与 send/recv 共享同一组变量，
 * 一次非原子的 16 位读本身在 M4 上虽不会被打断（对齐的半字访存是原子的），
 * 但"读到 count 之后、依据它做决策之前"存在窗口，
 * 因此把读操作一并纳入临界区，让查询结果具有决策意义。
 *--------------------------------------------------------------------------*/
uint16_t scheduler_queue_count(const sched_queue_t *q)
{
    uint16_t n;

    if (q == NULL)
        return 0;

    __disable_irq();
    n = q->count;
    __enable_irq();

    return n;
}

uint16_t scheduler_queue_space(const sched_queue_t *q)
{
    uint16_t n;

    if (q == NULL)
        return 0;

    __disable_irq();
    n = (uint16_t)(q->capacity - q->count);
    __enable_irq();

    return n;
}

/*---------------------------------------------------------------------------
 * 清空队列
 *--------------------------------------------------------------------------*/
void scheduler_queue_reset(sched_queue_t *q)
{
    if (q == NULL)
        return;

    __disable_irq();
    q->head  = 0U;
    q->tail  = 0U;
    q->count = 0U;
    __enable_irq();
}
