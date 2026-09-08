#include "scheduler_sem.h"
#include "scheduler.h"
#include "port_mem.h"

/*===========================================================================
 * scheduler_sem 实现
 *
 * 结构极简：只有一个计数值和一个上限。
 * "谁在等待"这件事不记录在信号量里 —— 阻塞任务挂在内核的阻塞链上，
 * 靠 TCB 的 wait_obj 指向本信号量来关联。scheduler_wake_one(sem) 会去
 * 阻塞链上找出等待本信号量的任务，无需本模块维护等待队列。
 *=========================================================================*/

/*---------------------------------------------------------------------------
 * 创建信号量
 *--------------------------------------------------------------------------*/
sched_sem_t *scheduler_sem_create(uint16_t init, uint16_t max)
{
    sched_sem_t *sem;

    if (max == 0U)
        return NULL;

    if (init > max)
        init = max;                     /* 初值不得超过上限 */

    __disable_irq();

    sem = (sched_sem_t *)scheduler_port_mem_alloc(sizeof(sched_sem_t));
    if (sem == NULL)
    {
        __enable_irq();
        return NULL;
    }

    sem->count = init;
    sem->max   = max;

    __enable_irq();

    return sem;
}

/*---------------------------------------------------------------------------
 * 获取（P 操作）
 *
 * 与队列的 recv_block 同构：判空与登记阻塞必须在同一个临界区内，
 * 否则中断里的 give 会落空（丢失唤醒）。
 *--------------------------------------------------------------------------*/
int scheduler_sem_take(sched_sem_t *sem, uint32_t timeout_ms)
{
    if (sem == NULL)
        return 0;

    __disable_irq();

    if (sem->count > 0U)                /* 有令牌：直接取走，不阻塞 */
    {
        sem->count--;
        __enable_irq();
        return 1;
    }

    /* 调度器尚未启动：无法挂起，退化为非阻塞 */
    if (current_tcb == NULL)
    {
        __enable_irq();
        return 0;
    }

    scheduler_wait_prepare(sem, timeout_ms);
    __enable_irq();

    /* ===== 任务在此挂起；被唤醒或从超时返回后从下一行继续 =====
       被唤醒意味着 give 方已把令牌直接过继给本任务（count 未增加），
       所以这里不需要再去减 count，直接按"拿到了"返回。 */
    return (current_tcb->wait_result != 0U) ? 1 : 0;
}

/*---------------------------------------------------------------------------
 * 释放（V 操作）
 *
 * 可在 ISR 中调用。
 *--------------------------------------------------------------------------*/
void scheduler_sem_give(sched_sem_t *sem)
{
    if (sem == NULL)
        return;

    __disable_irq();

    /* 有任务在等 → 令牌直接过继给它，count 保持不变。
       这样接收方醒来时不必再减一次 count，也就不会出现
       "被唤醒却抢不到令牌"的窗口。 */
    if (scheduler_wake_one(sem) != 0U)
    {
        __enable_irq();
        return;
    }

    /* 无人等待 → 计数值累加，达到上限则本次 give 被丢弃（饱和） */
    if (sem->count < sem->max)
        sem->count++;

    __enable_irq();
}

/*---------------------------------------------------------------------------
 * 当前计数值
 *--------------------------------------------------------------------------*/
uint16_t scheduler_sem_count(sched_sem_t *sem)
{
    uint16_t n;

    if (sem == NULL)
        return 0;

    __disable_irq();
    n = sem->count;
    __enable_irq();

    return n;
}
