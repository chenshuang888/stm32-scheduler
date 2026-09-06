#include "scheduler_queue.h"
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
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * 临界区：保存/恢复 PRIMASK
 *
 * 为什么不是裸的 __disable_irq() / __enable_irq()：
 *   裸 __enable_irq() 会无条件开中断。若调用者本身已在临界区内（例如将来
 *   在内核的关中断路径里入队），返回时会被本模块提前打开，破坏外层保护。
 *   保存 PRIMASK 再按原值恢复，就自然支持嵌套。
 *
 *   __get_PRIMASK() 返回 0 表示进临界区前中断是开的（由本模块负责重开），
 *   返回 1 表示原本就关着（本模块不动，交给外层）。
 *
 * 为什么足以保护队列：
 *   head/tail/count 的并发只可能来自"任务 vs 中断"。关中断后同优先级任务
 *   不会抢占、中断也不会插入，读改写序列天然原子。这也是本模块暂时不需要
 *   调度器提供正式临界区 API 的原因。
 *-------------------------------------------------------------------------*/
static uint32_t q_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void q_unlock(uint32_t primask)
{
    if (primask == 0U)
        __enable_irq();
}

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
    uint32_t       primask;

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

    primask = q_lock();

    buf = (uint8_t *)scheduler_port_mem_alloc(total);
    if (buf == NULL)
    {
        q_unlock(primask);
        return NULL;
    }

    q = (sched_queue_t *)scheduler_port_mem_alloc(sizeof(sched_queue_t));
    if (q == NULL)
    {
        q_unlock(primask);
        return NULL;                       /* 存储区已泄漏，但池本就不支持回收 */
    }

    q->buf       = buf;
    q->item_size = (uint16_t)aligned;
    q->capacity  = capacity;
    q->head      = 0U;
    q->tail      = 0U;
    q->count     = 0U;

    q_unlock(primask);

    return q;
}

/*---------------------------------------------------------------------------
 * 入队
 *--------------------------------------------------------------------------*/
int scheduler_queue_send(sched_queue_t *q, const void *item)
{
    uint32_t primask;
    uint8_t *dst;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    primask = q_lock();

    if (q->count >= q->capacity)           /* 满：丢弃新消息 */
    {
        q_unlock(primask);
        return 0;
    }

    dst = q->buf + (uint32_t)q->tail * q->item_size;
    memcpy(dst, item, q->item_size);

    q->tail  = q_next(q, q->tail);
    q->count = (uint16_t)(q->count + 1U);

    q_unlock(primask);

    return 1;
}

/*---------------------------------------------------------------------------
 * 出队
 *--------------------------------------------------------------------------*/
int scheduler_queue_recv(sched_queue_t *q, void *item)
{
    uint32_t       primask;
    const uint8_t *src;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    primask = q_lock();

    if (q->count == 0U)                    /* 空：非阻塞，立即返回 */
    {
        q_unlock(primask);
        return 0;
    }

    src = q->buf + (uint32_t)q->head * q->item_size;
    memcpy(item, src, q->item_size);

    q->head  = q_next(q, q->head);
    q->count = (uint16_t)(q->count - 1U);

    q_unlock(primask);

    return 1;
}

/*---------------------------------------------------------------------------
 * 窥视队头（不移除）
 *--------------------------------------------------------------------------*/
int scheduler_queue_peek(sched_queue_t *q, void *item)
{
    uint32_t       primask;
    const uint8_t *src;

    if (q == NULL || q->buf == NULL || item == NULL)
        return 0;

    primask = q_lock();

    if (q->count == 0U)
    {
        q_unlock(primask);
        return 0;
    }

    src = q->buf + (uint32_t)q->head * q->item_size;
    memcpy(item, src, q->item_size);

    q_unlock(primask);

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
    uint32_t primask;
    uint16_t n;

    if (q == NULL)
        return 0;

    primask = q_lock();
    n = q->count;
    q_unlock(primask);

    return n;
}

uint16_t scheduler_queue_space(const sched_queue_t *q)
{
    uint32_t primask;
    uint16_t n;

    if (q == NULL)
        return 0;

    primask = q_lock();
    n = (uint16_t)(q->capacity - q->count);
    q_unlock(primask);

    return n;
}

/*---------------------------------------------------------------------------
 * 清空队列
 *--------------------------------------------------------------------------*/
void scheduler_queue_reset(sched_queue_t *q)
{
    uint32_t primask;

    if (q == NULL)
        return;

    primask = q_lock();
    q->head  = 0U;
    q->tail  = 0U;
    q->count = 0U;
    q_unlock(primask);
}
