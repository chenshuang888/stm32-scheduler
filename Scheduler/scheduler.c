#include "scheduler.h"
#include "port_timer.h"
#include "port_mem.h"

/*===========================================================================
 * 全局指针
 *
 * 说明：本文件是内核核心，不 include / 不感知任何用户模块
 *       （LED / UART / ICM）。用户任务由各模块自行调用
 *       scheduler_task_create() 创建，与调度器解耦。
 *
 *       内核只负责创建自身的兜底任务 IDLE。任务数量不受编译期
 *       常量限制，上限取决于内存池剩余空间。
 *=========================================================================*/

/* 当前正在运行的任务指针，port_asm.s 通过此全局变量访问 TCB。
   注意：调度器启动前保持 NULL，"current_tcb == NULL"即代表尚未启动，
        need_context_switch() 依赖这一点避免在 PSP 未初始化时触发 PendSV。 */
TCB_t *current_tcb = NULL;

/*===========================================================================
 * 任务链表头
 *
 * 就绪链 (ready_head)
 *   按优先级升序排列，同优先级内 FIFO。
 *   链头恒为"最高优先级中最早就绪者"，故选择任务为 O(1)。
 *   只放 READY 状态的任务 —— 正在运行者已被摘出。
 *
 * 阻塞链 (blocked_head)
 *   只放 BLOCKED 状态的任务。无序，每次 tick 全量遍历递减延时。
 *=========================================================================*/
static TCB_t *ready_head   = NULL;
static TCB_t *blocked_head = NULL;

/*===========================================================================
 * 链表操作（内部使用）
 *
 * 全部为单向链表，用"指向指针的指针"遍历，省去对头节点的特殊分支。
 * 调用方必须保证 tcb 当前不在目标链表中，否则会成环。
 *=========================================================================*/

/* 插入就绪链：按优先级升序，同优先级插到组末尾
 *
 *   条件用 "<=" 而非 "<" —— 同优先级的既有节点会被跳过，
 *   新节点因此落在同优先级组的末尾，形成 FIFO。
 *   这正是"同优先级轮转"的实现：让出的任务重新插入时排到队尾。
 */
static void ready_list_insert(TCB_t *tcb)
{
    TCB_t **pp = &ready_head;

    while (*pp != NULL && (*pp)->prio <= tcb->prio)
        pp = &(*pp)->next;

    tcb->next = *pp;
    *pp       = tcb;
}

/* 从就绪链摘除 */
static void ready_list_remove(TCB_t *tcb)
{
    TCB_t **pp = &ready_head;

    while (*pp != NULL && *pp != tcb)
        pp = &(*pp)->next;

    if (*pp != NULL)
        *pp = tcb->next;
}

/* 插入阻塞链：头插即可，无需排序（tick 会全量遍历） */
static void blocked_list_insert(TCB_t *tcb)
{
    tcb->next    = blocked_head;
    blocked_head = tcb;
}

/* 从阻塞链摘除
 *
 * 注意：调用点（scheduler_tick）正在遍历本链，
 *       因此必须先保存 next 再摘除，否则遍历指针失效。
 */
static void blocked_list_remove(TCB_t *tcb)
{
    TCB_t **pp = &blocked_head;

    while (*pp != NULL && *pp != tcb)
        pp = &(*pp)->next;

    if (*pp != NULL)
        *pp = tcb->next;
}

/*---------------------------------------------------------------------------
 * 唤醒一个阻塞任务（内部函数）
 *
 *   result : 唤醒原因，1 = 等到了 / 延时到，0 = 超时
 *
 * 把 wait_obj 置空是刻意的一步：它兼作"已唤醒"标记。
 * 若之后 tick 又因超时扫到这个任务，会因 wait_obj == NULL 而不再重复唤醒
 * （任务此时已不在阻塞链上，扫描本就碰不到它，置空是双保险）。
 *--------------------------------------------------------------------------*/
static void sched_wake(TCB_t *tcb, uint8_t result)
{
    blocked_list_remove(tcb);
    tcb->wait_obj    = NULL;
    tcb->wait_result = result;
    tcb->state       = TASK_READY;
    ready_list_insert(tcb);
}

/*===========================================================================
 * 内部函数
 *=========================================================================*/

/* 任务意外退出时的保护：正常情况下任务不应从 while(1) 退出 */
static void task_exit_error(void)
{
    __disable_irq();
    while (1);
}

/*---------------------------------------------------------------------------
 * 空闲任务：优先级最低，仅在所有用户任务均 BLOCKED 时运行
 *
 * 注意：Idle 永不调用 scheduler_delay()，因此永远保持 READY 状态。
 *       这保证调度器在任何时刻都能选出可运行的任务，
 *       pick_highest_ready() 的返回值因此永远不为 NULL。
 *-------------------------------------------------------------------------*/
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
 * 栈帧内存布局（stack_ptr 指向最高地址，向低地址增长）：
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
    /*------------------------------------------------------------------------
     * 栈顶定位与 8 字节对齐（关键！）
     *
     * 先指向数组末尾之后，再向下对齐到 8 字节，记为 sp_initial。
     * 配合下面的"先减后写"（*--sp），任务开始执行时的 PSP 恰好等于
     * sp_initial，满足 AAPCS 对 8 字节对齐的要求。
     *
     * !! 为什么必须 8 字节对齐 !!
     *    AAPCS 规定函数调用时 SP 必须 8 字节对齐。若不对齐，
     *    printf 的 %f 参数（double，8 字节）的写入与读取位置会系统性错开，
     *    打印出 2.68e155 这类天文数字。
     *    旧写法（先写后减、且 R4 用 *sp= 不减）会让 PSP = sp_initial + 4，
     *    只有 4 字节对齐，正是踩了这个坑。
     *
     *    注意：裸机阶段（调度器启动前）用 MSP，由启动文件保证 8 对齐，
     *    所以 ICM_CalibrateGyro 里的 %.2f 一直正常 —— 这个对照也是
     *    当初定位本问题的关键线索。
     *----------------------------------------------------------------------*/
    uint32_t *sp = (uint32_t *)(((uint32_t)(stack_base + stack_words)) & ~7UL);

    /* ---- 魔数填充：必须在构造栈帧之前完成 ----
       填满后，未被任务触碰过的区域将长期保持该魔数，
       供 scheduler_stack_free() 计算栈高水位。 */
    for (uint32_t i = 0; i < stack_words; i++)
        stack_base[i] = STACK_FILL_PATTERN;

    /* ---- 硬件帧（高地址端，CPU 在异常入口/出口自动操作） ---- */
    *--sp = 0x01000000UL;               /* xPSR: Thumb bit (bit24)    */
    *--sp = (uint32_t)func;             /* PC:   任务入口地址          */
    *--sp = (uint32_t)task_exit_error;  /* LR:   任务意外 return 时调用 */
    *--sp = 0UL;                        /* R12                         */
    *--sp = 0UL;                        /* R3                          */
    *--sp = 0UL;                        /* R2                          */
    *--sp = 0UL;                        /* R1                          */
    *--sp = 0UL;                        /* R0                          */

    /* ---- 软件帧（低地址端，PendSV_Handler 手动操作） ---- */
    *--sp = 0xFFFFFFFDUL;               /* LR: EXC_RETURN (Thread/PSP/无FPU) */
    *--sp = 0UL;                        /* R11 */
    *--sp = 0UL;                        /* R10 */
    *--sp = 0UL;                        /* R9  */
    *--sp = 0UL;                        /* R8  */
    *--sp = 0UL;                        /* R7  */
    *--sp = 0UL;                        /* R6  */
    *--sp = 0UL;                        /* R5  */
    *--sp = 0UL;                        /* R4  ← stack_ptr 指向此处 */

    tcb->stack_ptr   = sp;              /* = sp_initial - 68 */
    tcb->stack_base  = stack_base;
    tcb->stack_words = stack_words;
}

/*---------------------------------------------------------------------------
 * scheduler_stack_free：查询指定任务的栈剩余量（高水位）
 *
 * 从栈底（低地址）向上扫描，统计连续保持 STACK_FILL_PATTERN 的字数。
 * 栈向低地址增长，因此低地址端残留的魔数 = 从未被使用过的栈空间。
 *
 * 返回：剩余未使用空间（单位：字）。0 表示该任务已把栈用到栈底（溢出）。
 *
 * 用法：让系统跑一段时间并覆盖所有代码路径后调用，
 *       峰值用量 = handle->stack_words - scheduler_stack_free(handle)。
 *-------------------------------------------------------------------------*/
uint32_t scheduler_stack_free(TCB_t *handle)
{
    uint32_t *base;
    uint32_t  words;
    uint32_t  n = 0;

    if (handle == NULL)
        return 0;

    base  = handle->stack_base;
    words = handle->stack_words;

    while (n < words && base[n] == STACK_FILL_PATTERN)
        n++;

    return n;
}

/*---------------------------------------------------------------------------
 * pick_highest_ready：选出下一个应运行的任务
 *
 * 就绪链已按优先级升序排列、同优先级内 FIFO，故链头就是答案，O(1)。
 * 返回 NULL 不会发生 —— Idle 永不阻塞，始终挂在就绪链上。
 *-------------------------------------------------------------------------*/
static TCB_t *pick_highest_ready(void)
{
    return ready_head;
}

/*---------------------------------------------------------------------------
 * need_context_switch：判断当前是否需要进行一次上下文切换
 *
 * 调度策略 —— 抢占的唯一正当理由是"优先级更高"：
 *
 *   1. 当前任务已 BLOCKED（刚调用 scheduler_delay）
 *      → 必须让出，否则 delay 会退化成空操作，任务继续往下跑。
 *
 *   2. 存在优先级更高的 READY 任务
 *      → 抢占。Idle 优先级最低，所以"Idle 让位给用户任务"是这条规则的
 *         自然推论，无需为 Idle 写任何特判。
 *
 *   3. 同优先级的任务就绪
 *      → 不抢占。任务只在主动让出 CPU 时才切换，因此运行期间不会被打断，
 *         printf 这类多步骤操作不可能被切走，从根上避免多任务重入。
 *
 * 这样调度器在同优先级下表现为协作式，跨优先级下才是抢占式。
 *-------------------------------------------------------------------------*/
static uint8_t need_context_switch(void)
{
    if (current_tcb == NULL)
        return 0;

    /* 情形 1：当前任务已阻塞，必须让出 */
    if (current_tcb->state == TASK_BLOCKED)
        return 1;

    /* 情形 2：有更高优先级的任务就绪，抢占。
       就绪链按优先级升序，只需比较链头即可，无需遍历。 */
    if (ready_head != NULL && ready_head->prio < current_tcb->prio)
        return 1;

    /* 情形 3：同优先级或无更高优先级任务就绪 —— 不切换 */
    return 0;
}

/*---------------------------------------------------------------------------
 * 动态创建任务
 *
 * 流程：
 *   1. 校验参数
 *   2. 从内存池分配栈与 TCB（先大后小，最小化失败时的浪费）
 *   3. 初始化栈帧
 *   4. 挂入就绪链
 *   5. 若新任务优先级更高，触发抢占
 *
 * 关中断保护：整个创建过程会同时修改内存池游标与就绪链，
 * 必须保证不被时基中断打断，否则中断中的调度逻辑会读到半成品。
 *-------------------------------------------------------------------------*/
TCB_t *scheduler_task_create(void (*func)(void), const char *name,
                             uint8_t prio, uint32_t stack_words)
{
    uint32_t *stack;
    TCB_t    *tcb;

    /* --- 1. 参数校验 --- */
    if (func == NULL || stack_words < MIN_STACK_WORDS)
        return NULL;

    __disable_irq();

    /* --- 2. 分配内存：先大（栈）后小（TCB）---
       池不支持释放，分配一半失败无法回滚。
       栈失败时不会占用 TCB；TCB 失败时仅浪费一块栈空间，损失最小。 */
    stack = scheduler_port_mem_alloc(stack_words * 4);
    if (stack == NULL)
    {
        __enable_irq();
        return NULL;
    }

    tcb = scheduler_port_mem_alloc(sizeof(TCB_t));
    if (tcb == NULL)
    {
        __enable_irq();
        return NULL;
    }

    /* --- 3. 初始化栈帧与 TCB 字段 --- */
    tcb_stack_init(tcb, stack, stack_words, func);
    tcb->prio        = prio;
    tcb->state       = TASK_READY;
    tcb->delay_ticks = 0;
    tcb->name        = name;
    tcb->next        = NULL;
    tcb->wait_obj    = NULL;      /* 无等待对象 = 延时阻塞语义 */
    tcb->wait_result = 0;

    /* --- 4. 挂入就绪链 --- */
    ready_list_insert(tcb);

    /* --- 5. 抢占判断 ---
       current_tcb != NULL 表明调度器已启动（PSP 已生效），
       此时若新任务优先级更高，立即触发 PendSV 抢占。
       调度器启动前（如创建 Idle）不触发 —— 那时 PSP 尚未初始化，
       触发 PendSV 必然 BusFault。 */
    if (current_tcb != NULL && tcb->prio < current_tcb->prio)
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    __enable_irq();

    return tcb;
}

/*===========================================================================
 * 公共 API 实现
 *=========================================================================*/

void scheduler_init(void)
{
    scheduler_port_mem_init();

    /* 链表头初始化 */
    ready_head   = NULL;
    blocked_head = NULL;

    /* 创建内核兜底任务 IDLE。
       它是"永远就绪"的最低位任务 —— 所有用户任务都阻塞时由它运行
       （执行 WFI 等待）。缺失后一旦无用户任务就绪，调度器将无人可选。
       用户任务不由本文件创建：各模块在调用 scheduler_init() 之后、
       scheduler_start() 之前，自行调用 scheduler_task_create()。 */
    if (scheduler_task_create(idle_task_func, "IDLE",
                              TASK_PRIO_IDLE, IDLE_STACK_SIZE) == NULL)
    {
        Error_Handler();
    }

    /* PendSV 设为最低优先级（15），确保不打断任何外设中断 */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /* 配置调度器时基（TIM6 1kHz）。
       此处只完成外设配置并使能 NVIC，计数器保持停止（CEN=0）；
       真正的计数启动由 port_asm.s 的 SVC_Handler 在切换到第一个任务前完成，
       确保 PSP 生效之后才可能产生时基中断与 PendSV。 */
    scheduler_port_timer_init();

    /* 注意：此处刻意不设置 current_tcb。
       保持 NULL 表示"调度器尚未启动"，"选第一个任务"推迟到
       scheduler_start() 完成。这样 current_tcb==NULL 成为区分
       "初始化期"与"运行期"的天然判据，抢占判断才不会在
       PSP 未初始化时误触发 PendSV。 */
}

/*---------------------------------------------------------------------------
 * scheduler_tick：由 port_timer.c 的 TIM6 中断每 1ms 调用
 * 递减所有 BLOCKED 任务的延时计数，到期后标记为 READY。
 *
 * Idle 永不 BLOCKED，在循环中被自然跳过，无需特判。
 *--------------------------------------------------------------------------*/
void scheduler_tick(void)
{
    TCB_t  *p;
    uint8_t need = 0;

    /* 调度器尚未启动：不做任何调度决策 */
    if (current_tcb == NULL)
        return;

    /* 只遍历阻塞链 —— 就绪任务无需处理，运行中任务不在任何链上 */
    p = blocked_head;
    while (p != NULL)
    {
        TCB_t *next = p->next;      /* 必须先保存：p 到期后会被移出本链 */

        if (p->delay_ticks > 0)
        {
            if (--p->delay_ticks == 0)
            {
                /* wait_obj 非空 → 这是"带超时的事件等待"，到点算超时（0）；
                   否则是普通延时阻塞，到点是正常唤醒（1）。

                   注意：事件等待且 delay_ticks == 0（永久等待）时，
                   外层 if (p->delay_ticks > 0) 不成立，会被自然跳过 ——
                   永久等待的任务只由 scheduler_wake_one() 唤醒，
                   绝不会被 tick 误唤醒。 */
                sched_wake(p, (p->wait_obj != NULL) ? 0U : 1U);
                need = 1;
            }
        }

        p = next;
    }

    /* 只在真正需要切换时才置位 PendSV。
       无任务到期且无更高优先级任务就绪时完全不进异常 ——
       这样 Idle 期间的 __WFI() 不会被每 1ms 唤醒一次，低功耗才有意义。 */
    if (need && need_context_switch())
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/*---------------------------------------------------------------------------
 * scheduler_switch_context：由 PendSV_Handler 汇编调用
 * 只负责"切给谁"，"要不要切"由 need_context_switch() 决定。
 *--------------------------------------------------------------------------*/
void scheduler_switch_context(void)
{
    TCB_t *prev = current_tcb;
    TCB_t *next;

    /* 处理前任任务：
         RUNNING（被抢占，或 Idle 给用户任务让位）
           → 改回 READY 并插回就绪链。
             由于插入是"同优先级排到组末尾"，这就实现了轮转公平性。
         BLOCKED（主动调用 scheduler_delay）
           → 已在 scheduler_delay 中移入阻塞链，此处不再处理，
             避免同一个 TCB 被挂到两条链上导致链表成环。 */
    if (prev->state == TASK_RUNNING)
    {
        prev->state = TASK_READY;
        ready_list_insert(prev);
    }

    /* 选出继任者：就绪链头（最高优先级 + FIFO），并从链上摘下 */
    next = pick_highest_ready();
    if (next != NULL)
    {
        ready_list_remove(next);
        next->state = TASK_RUNNING;
        current_tcb = next;
    }
    /* next == NULL 不可达：Idle 永不阻塞，始终在就绪链上 */
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
 *
 * 本函数负责把任务从就绪链迁移到阻塞链，因此 scheduler_switch_context()
 * 中不再处理 BLOCKED 情形，确保每个 TCB 任一时刻只挂在一条链上。
 *
 * 这里无条件置位 PendSV：任务已主动进入 BLOCKED，切换是必然的，
 * 无需再走 need_context_switch() 判断。
 *--------------------------------------------------------------------------*/
void scheduler_delay(uint32_t ms)
{
    if (ms == 0) return;

    __disable_irq();
    current_tcb->state       = TASK_BLOCKED;
    current_tcb->delay_ticks = ms;

    /* 从就绪链摘除并挂入阻塞链；
       scheduler_switch_context() 只会处理仍处于 RUNNING 的任务，
       故此处必须先迁移，否则 BLOCKED 任务会被误插回就绪链 */
    ready_list_remove(current_tcb);
    blocked_list_insert(current_tcb);

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __enable_irq();
}

/*---------------------------------------------------------------------------
 * 事件阻塞：把当前任务登记为"等待 obj"并阻塞
 *
 * !! 自身不关中断 —— 调用者必须已经处于临界区内 !!
 *
 * 为什么"判断条件不满足"与"登记阻塞"必须在同一个临界区内：
 *   这是经典的丢失唤醒（lost wakeup）问题。若两步之间中断被打开，
 *   生产者可能已经放好数据并尝试唤醒，而本任务还没挂上阻塞链 ——
 *   唤醒落空；随后本任务才阻塞，数据躺在队列里却再也不会有人来唤醒。
 *   把两步关在同一段临界区内，这个窗口就被彻底堵死了。
 *
 *   timeout_ms = 0 → 永久等待（delay_ticks 为 0，tick 不会递减它）
 *   timeout_ms > 0 → 限时等待，到期由 tick 以"超时"唤醒
 *
 * 任务被唤醒后，从本函数的返回处继续执行，读自己的
 * current_tcb->wait_result 即可区分"等到了(1)"还是"超时(0)"。
 *--------------------------------------------------------------------------*/
void scheduler_wait_prepare(void *obj, uint32_t timeout_ms)
{
    /* 调度器尚未启动（current_tcb == NULL，PSP 未生效）：
       无法阻塞，也不该置位 PendSV，直接返回。
       调用方会看到 wait_result 仍为 0，自行按"没等到"处理。 */
    if (current_tcb == NULL)
        return;

    current_tcb->wait_obj    = obj;
    current_tcb->wait_result = 0;
    current_tcb->delay_ticks = timeout_ms;
    current_tcb->state       = TASK_BLOCKED;

    /* 与 scheduler_delay() 同样处理：先从就绪链摘下，再挂进阻塞链，
       保证任一时刻一个 TCB 只挂在一条链上（否则链表会成环）。 */
    ready_list_remove(current_tcb);
    blocked_list_insert(current_tcb);

    /* 已主动进入 BLOCKED，切换是必然的，无需走 need_context_switch()。
       PendSV 会等调用者开中断之后才真正执行。 */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/*---------------------------------------------------------------------------
 * 事件唤醒：唤醒等待 obj 的优先级最高的那个任务
 *
 * !! 自身不关中断 —— 调用者必须已经处于临界区内 !!
 *    在 ISR 中调用时，同样建议在调用前后自行关中断。
 *
 * obj 通常就是队列控制块指针 —— 不必为"等待对象"引入新的结构体。
 *
 * 只唤醒一个：一条消息 / 一个令牌只应有一个接收者拿到。
 * 阻塞链通常只有 1~3 个任务，线性扫描找最高优先级者的开销可忽略，
 * 比给每个队列单独维护一条等待链（TCB 要再加一个链表指针）划算得多。
 *
 * 返回：1 = 唤醒了一个任务；0 = 没人在等 obj。
 *   信号量需要这个返回值 —— give 时若已把令牌过继给等待者，
 *   就不能再累加计数值，否则同一个令牌会被算两次。
 *--------------------------------------------------------------------------*/
uint8_t scheduler_wake_one(void *obj)
{
    TCB_t *p;
    TCB_t *best = NULL;

    if (obj == NULL)
        return 0;

    for (p = blocked_head; p != NULL; p = p->next)
    {
        if (p->wait_obj == obj && (best == NULL || p->prio < best->prio))
            best = p;
    }

    if (best == NULL)
        return 0;                       /* 没有任务在等这个对象 */

    sched_wake(best, 1);

    /* 被唤醒者优先级更高 → 抢占。
       同优先级不抢占，遵循"跨优先级抢占、同优先级协作"的既有策略。 */
    if (current_tcb != NULL && best->prio < current_tcb->prio)
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    return 1;
}

/*---------------------------------------------------------------------------
 * scheduler_start：选出第一个任务并启动调度器，不返回
 *
 * "选第一个任务"在此完成而非 scheduler_init()：
 *   这样 init 结束后 current_tcb 仍为 NULL，成为"调度器尚未运行"的判据。
 *   need_context_switch() 依赖它避免在 PSP 未初始化时触发 PendSV。
 *
 * SVC_Handler（定义在 port_asm.s）负责切换到该任务并启动时基，不会返回。
 *--------------------------------------------------------------------------*/
void scheduler_start(void)
{
    /* 选出首个任务：就绪链头即最高优先级者，并从链上摘下 */
    current_tcb = pick_highest_ready();
    if (current_tcb != NULL)
    {
        ready_list_remove(current_tcb);
        current_tcb->state = TASK_RUNNING;
    }

    __asm volatile ("SVC #0");
    while (1);  /* 永不执行 */
}
