#include "scheduler.h"
#include "port_timer.h"
#include "led.h"
#include "uart.h"
#include "icm20608.h"

/*===========================================================================
 * 用户任务注册表
 * 在此处添加/删除任务，不需要改动其他地方
 *=========================================================================*/
#define USER_TASK_COUNT   3

static const struct {
    void       (*func)(void);
    const char  *name;
} user_task_table[USER_TASK_COUNT] = {
    { Led_Task,  "LED"  },
    { Uart_Task, "UART" },
    { ICM_Task,  "ICM"  },
};

/*===========================================================================
 * TCB 池（用户任务 + 1 个 Idle 任务）
 * 栈空间与 TCB 分离，可为不同任务配置不同栈大小
 *=========================================================================*/
#define TOTAL_TASK_COUNT  (USER_TASK_COUNT + 1)   /* 最后一个是 Idle */

static TCB_t    tcb_pool[TOTAL_TASK_COUNT];
static uint32_t user_stacks[USER_TASK_COUNT][TASK_STACK_SIZE];
static uint32_t idle_stack[IDLE_STACK_SIZE];

/* 当前正在运行的任务指针，port_asm.s 通过此全局变量访问 TCB */
TCB_t *current_tcb = NULL;

/*===========================================================================
 * 内部函数
 *=========================================================================*/

/* 任务意外退出时的保护：正常情况下任务不应从 while(1) 退出 */
static void task_exit_error(void)
{
    __disable_irq();
    while (1);
}

/* 空闲任务：所有用户任务均 BLOCKED 时执行，WFI 节省功耗 */
static void idle_task_func(void)
{
    while (1)
    {
        __WFI();
    }
}

/*---------------------------------------------------------------------------
 * 初始化单个任务的栈帧
 *
 * 栈帧内存布局（stack_ptr 指向最低地址，向高地址增长）：
 *
 *  stack_ptr →  [R4  = 0      ]  ┐
 *               [R5  = 0      ]  │
 *               [R6  = 0      ]  │  软件帧：PendSV_Handler 手动保存/恢复
 *               [R7  = 0      ]  │  共 9 个字（R4-R11 + EXC_RETURN）
 *               [R8  = 0      ]  │
 *               [R9  = 0      ]  │
 *               [R10 = 0      ]  │
 *               [R11 = 0      ]  │
 *               [LR  = 0xFFFFFFFD] ← EXC_RETURN (Thread/PSP/无FPU)  ┘
 *               [R0  = 0      ]  ┐
 *               [R1  = 0      ]  │
 *               [R2  = 0      ]  │  硬件帧：异常入口 CPU 自动压栈
 *               [R3  = 0      ]  │           异常返回 CPU 自动弹栈
 *               [R12 = 0      ]  │  共 8 个字
 *               [LR  = exit   ]  │
 *               [PC  = func   ]  │
 *               [xPSR= 0x01000000] ← Thumb 位                       ┘
 *-------------------------------------------------------------------------*/
static void tcb_stack_init(TCB_t *tcb, uint32_t *stack_base,
                           uint32_t stack_words, void (*func)(void))
{
    uint32_t *sp = stack_base + stack_words - 1;

    /* 对齐到 8 字节（AAPCS 要求） */
    sp = (uint32_t *)((uint32_t)sp & ~7UL);

    /* ---- 硬件帧（高地址端，CPU 在异常入口/出口自动操作） ---- */
    *sp-- = 0x01000000UL;               /* xPSR: Thumb bit (bit24)    */
    *sp-- = (uint32_t)func;             /* PC:   任务入口地址          */
    *sp-- = (uint32_t)task_exit_error;  /* LR:   任务意外 return 时调用 */
    *sp-- = 0UL;                        /* R12                         */
    *sp-- = 0UL;                        /* R3                          */
    *sp-- = 0UL;                        /* R2                          */
    *sp-- = 0UL;                        /* R1                          */
    *sp-- = 0UL;                        /* R0                          */

    /* ---- 软件帧（低地址端，PendSV_Handler 手动操作） ---- */
    *sp-- = 0xFFFFFFFDUL;               /* LR: EXC_RETURN (Thread/PSP/无FPU) */
    *sp-- = 0UL;                        /* R11 */
    *sp-- = 0UL;                        /* R10 */
    *sp-- = 0UL;                        /* R9  */
    *sp-- = 0UL;                        /* R8  */
    *sp-- = 0UL;                        /* R7  */
    *sp-- = 0UL;                        /* R6  */
    *sp-- = 0UL;                        /* R5  */
    *sp   = 0UL;                        /* R4  ← stack_ptr 指向此处 */

    tcb->stack_ptr = sp;
}

/*===========================================================================
 * 公共 API 实现
 *=========================================================================*/

void scheduler_init(void)
{
    /* 初始化用户任务 */
    for (int i = 0; i < USER_TASK_COUNT; i++)
    {
        tcb_stack_init(&tcb_pool[i],
                       user_stacks[i], TASK_STACK_SIZE,
                       user_task_table[i].func);
        tcb_pool[i].state       = TASK_READY;
        tcb_pool[i].delay_ticks = 0;
        tcb_pool[i].name        = user_task_table[i].name;
    }

    /* 初始化空闲任务（总在最后一个槽位） */
    tcb_stack_init(&tcb_pool[USER_TASK_COUNT],
                   idle_stack, IDLE_STACK_SIZE,
                   idle_task_func);
    tcb_pool[USER_TASK_COUNT].state       = TASK_READY;
    tcb_pool[USER_TASK_COUNT].delay_ticks = 0;
    tcb_pool[USER_TASK_COUNT].name        = "IDLE";

    /* PendSV 设为最低优先级（15），确保不打断任何外设中断 */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /* 配置调度器时基（TIM6 1kHz）。
       此处只完成外设配置并使能 NVIC，计数器保持停止（CEN=0）；
       真正的计数启动由 port_asm.s 的 SVC_Handler 在切换到第一个任务前完成，
       确保 PSP 生效之后才可能产生时基中断与 PendSV。 */
    scheduler_port_timer_init();

    /* 第一个运行的任务 */
    current_tcb        = &tcb_pool[0];
    current_tcb->state = TASK_RUNNING;
}

/*---------------------------------------------------------------------------
 * scheduler_tick：由 port_timer.c 的 TIM6 中断每 1ms 调用
 * 递减所有 BLOCKED 任务的延时计数，到期后标记为 READY。
 * 只有发生"任务延时到期"时才抢占；任务主动让出 CPU 由
 * scheduler_delay() 自行触发 PendSV，与本函数无关。
 *--------------------------------------------------------------------------*/
void scheduler_tick(void)
{
    uint8_t need_switch = 0;

    for (int i = 0; i < USER_TASK_COUNT; i++)
    {
        if (tcb_pool[i].state == TASK_BLOCKED && tcb_pool[i].delay_ticks > 0)
        {
            if (--tcb_pool[i].delay_ticks == 0)
            {
                tcb_pool[i].state = TASK_READY;
                need_switch       = 1;      /* 有任务到期，需要抢占 */
            }
        }
    }

    /* 仅在有任务到期时触发 PendSV。
       若无条件置位会导致：
         1) 每 1ms 强制轮转一次，破坏"阻塞才让出"的调度语义
         2) 全部任务阻塞时仍每 1ms 做一次 Idle→Idle 上下文空转，__WFI() 失效
         3) printf 等长耗时操作被中途切走，多任务输出在字节流层面交错 */
    if (need_switch)
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/*---------------------------------------------------------------------------
 * scheduler_switch_context：由 PendSV_Handler 汇编调用
 * 轮询选择下一个 READY 的用户任务；若无就绪任务则切换到 Idle
 *--------------------------------------------------------------------------*/
void scheduler_switch_context(void)
{
    /* 当前任务若仍是 RUNNING 状态，改为 READY（BLOCKED 则保持） */
    if (current_tcb->state == TASK_RUNNING)
        current_tcb->state = TASK_READY;

    /* 找到当前用户任务的下标（若当前是 Idle 则从 0 开始搜索） */
    int cur = 0;
    for (int i = 0; i < USER_TASK_COUNT; i++)
    {
        if (current_tcb == &tcb_pool[i]) { cur = i; break; }
    }

    /* 轮询查找下一个就绪的用户任务 */
    for (int i = 1; i <= USER_TASK_COUNT; i++)
    {
        int next = (cur + i) % USER_TASK_COUNT;
        if (tcb_pool[next].state == TASK_READY)
        {
            current_tcb        = &tcb_pool[next];
            current_tcb->state = TASK_RUNNING;
            return;
        }
    }

    /* 无就绪用户任务 → 切换到 Idle（Idle 永远 READY） */
    current_tcb        = &tcb_pool[USER_TASK_COUNT];
    current_tcb->state = TASK_RUNNING;
}

/*---------------------------------------------------------------------------
 * scheduler_delay：任务主动阻塞，让出 CPU
 *
 * 工作流程：
 *   1. 禁止中断，设置 BLOCKED 状态和延时计数
 *   2. 置位 PENDSVSET，开中断后 PendSV 立即触发
 *   3. PendSV 保存此处上下文，切换到其他任务
 *   4. ms 毫秒后 scheduler_tick 将此任务改为 READY
 *   5. 调度器再次选中此任务，从 PendSV 触发点恢复执行
 *   6. scheduler_delay 正常返回给调用者
 *--------------------------------------------------------------------------*/
void scheduler_delay(uint32_t ms)
{
    if (ms == 0) return;

    __disable_irq();
    current_tcb->state       = TASK_BLOCKED;
    current_tcb->delay_ticks = ms;
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __enable_irq();
}

/*---------------------------------------------------------------------------
 * scheduler_start：触发 SVC 异常，在异常上下文中完成首次任务启动
 * SVC_Handler（定义在 port_asm.s）负责切换到第一个任务，不会返回
 *--------------------------------------------------------------------------*/
void scheduler_start(void)
{
    __asm volatile ("SVC #0");
    while (1);  /* 永不执行 */
}
