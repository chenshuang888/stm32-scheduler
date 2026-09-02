#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "main.h"

/*===========================================================================
 * 任务栈大小（单位：字，1字 = 4字节）
 *  TASK_STACK_SIZE : 用户任务，2KB，ICM 任务有大量浮点运算和 printf，需要足够空间
 *  IDLE_STACK_SIZE : 空闲任务仅执行 __WFI()，256B 足够
 *=========================================================================*/
#define TASK_STACK_SIZE   512
#define IDLE_STACK_SIZE   64

typedef enum {
    TASK_READY     = 0,
    TASK_RUNNING   = 1,
    TASK_BLOCKED   = 2,
    TASK_SUSPENDED = 3,
} TaskState_t;

/*---------------------------------------------------------------------------
 * 任务控制块 (TCB)
 *
 * !! stack_ptr 必须是第一个成员 !!
 * port_asm.s 中的 PendSV_Handler 通过偏移 0 直接访问此字段。
 *--------------------------------------------------------------------------*/
typedef struct {
    uint32_t    *stack_ptr;
    TaskState_t  state;
    uint32_t     delay_ticks;
    const char  *name;
} TCB_t;

/*---------------------------------------------------------------------------
 * 对外 API
 *--------------------------------------------------------------------------*/
void scheduler_init(void);          /* 初始化 TCB 池，配置中断优先级      */
void scheduler_start(void);         /* 触发 SVC，启动第一个任务（不返回） */
void scheduler_delay(uint32_t ms);  /* 阻塞当前任务 ms 毫秒               */
void scheduler_tick(void);          /* 由 port_timer.c 的 TIM6 中断每 1ms 调用 */

/*---------------------------------------------------------------------------
 * 供 port_asm.s 访问
 *--------------------------------------------------------------------------*/
void   scheduler_switch_context(void);
extern TCB_t *current_tcb;

#endif /* __SCHEDULER_H */
