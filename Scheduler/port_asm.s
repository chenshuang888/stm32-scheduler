;;******************************************************************************
;; port_asm.s — Cortex-M4 上下文切换（Keil armasm 语法）
;;
;; 包含两个异常处理函数：
;;   PendSV_Handler  : 每次调度触发，完成任务上下文保存/恢复
;;   SVC_Handler     : 仅在启动时调用一次，启动第一个任务
;;
;; 注意：
;;   - stm32f4xx_it.c 中的 SVC_Handler 和 PendSV_Handler 必须删除，
;;     否则与此处定义冲突导致链接错误。
;;   - 此文件需要手动添加到 Keil 工程的 Source Group 中。
;;******************************************************************************

    AREA    |.text|, CODE, READONLY, ALIGN=2
    THUMB
    REQUIRE8
    PRESERVE8

    IMPORT  current_tcb
    IMPORT  scheduler_switch_context
    IMPORT  scheduler_port_timer_start

    EXPORT  PendSV_Handler
    EXPORT  SVC_Handler


;;==============================================================================
;; PendSV_Handler — 上下文切换核心
;;
;; 触发时机（由 C 侧的 need_context_switch() 决定）：
;;           1) 出现优先级更高的就绪任务 —— 抢占的唯一正当理由。
;;              Idle 优先级最低，因此"Idle 让位给用户任务"也走这条路径。
;;           2) 任务调用 scheduler_delay() 主动让出 CPU。
;;           同优先级的任务就绪不会触发切换。
;;           PendSV 以最低优先级在所有其他中断退出后执行。
;;
;; 栈操作均在任务的 PSP 上进行，MSP 保持给内核/中断使用。
;;
;; Cortex-M4 异常入口硬件自动保存（到 PSP）：
;;   xPSR, PC, LR, R12, R3, R2, R1, R0（若使用 FPU 还有 S0-S15, FPSCR）
;;
;; PendSV 额外手动保存/恢复：
;;   R4-R11, EXC_RETURN（LR）
;;   若任务使用了 FPU：S16-S31
;;==============================================================================
PendSV_Handler  PROC

    ;; 关中断，保护整个切换过程
    CPSID   I
    DSB
    ISB

    ;; 获取当前任务的 PSP
    MRS     R0, PSP
    ISB

    ;; 检查当前任务是否使用了 FPU（EXC_RETURN bit4=0 表示使用 FPU）
    TST     LR, #0x10
    IT      EQ
    VSTMDBEQ R0!, {S16-S31}         ;; 有 FPU 则手动保存 S16-S31

    ;; 保存 R4-R11 及 EXC_RETURN（LR）到当前任务栈
    ;; STMDB: 地址先递减再存储，R4 在最低地址，LR 在最高地址
    STMDB   R0!, {R4-R11, LR}

    ;; 将更新后的 PSP 写入当前 TCB 的 stack_ptr（TCB 第一个成员，偏移 0）
    LDR     R2, =current_tcb
    LDR     R1, [R2]
    STR     R0, [R1]                ;; current_tcb->stack_ptr = R0

    ;; 调用 C 函数选择下一个任务（会修改 current_tcb 和 LR）
    BL      scheduler_switch_context

    ;; 重新加载新任务的 TCB 和 PSP（BL 可能破坏 R0-R3）
    LDR     R2, =current_tcb
    LDR     R1, [R2]
    LDR     R0, [R1]                ;; R0 = 新任务的 stack_ptr

    ;; 恢复新任务的 R4-R11 及 EXC_RETURN（LR）
    ;; LDMIA: 从低地址到高地址加载，R4 先加载，LR 最后加载
    LDMIA   R0!, {R4-R11, LR}

    ;; 恢复新任务的 FPU 上下文（若新任务使用了 FPU）
    TST     LR, #0x10
    IT      EQ
    VLDMIAEQ R0!, {S16-S31}

    ;; 设置新任务的 PSP（R0 此时指向硬件帧底部，硬件返回时自动弹出）
    MSR     PSP, R0
    ISB

    ;; 开中断，通过 EXC_RETURN（LR）返回到新任务
    ;; 硬件自动从 PSP 弹出 R0-R3, R12, LR(task), PC, xPSR
    CPSIE   I
    DSB
    ISB
    BX      LR

    ENDP


;;==============================================================================
;; SVC_Handler — 启动第一个任务（仅调用一次）
;;
;; scheduler_start() 执行 "SVC #0" 触发此处理函数。
;; 利用异常上下文（Handler 模式），可以合法使用 EXC_RETURN（0xFFFFFFFD）
;; 通过 BX LR 返回到 Thread 模式并切换到 PSP，从而启动第一个任务。
;;
;; 同时负责启动调度器时基（TIM6）：
;;   TIM6 在 scheduler_init() 中已完成配置但 CEN=0，计数在此处启动。
;;   必须选在此刻的原因：本 Handler 优先级为 0，SysTick(15)/TIM6(15) 均
;;   无法抢占；且 PSP 已由下面的 MSR 设置完毕，此后产生的时基中断才允许
;;   触发 PendSV。若提前到 scheduler_start() 或 scheduler_init() 中启动，
;;   则 PSP 仍为 0，PendSV 的 STMDB 会写向地址 0 而必然 BusFault。
;;==============================================================================
SVC_Handler     PROC

    ;; 加载第一个任务的 TCB 和 stack_ptr
    LDR     R2, =current_tcb
    LDR     R1, [R2]
    LDR     R0, [R1]                ;; R0 = 第一个任务的 stack_ptr

    ;; 恢复软件帧：R4-R11, LR（LR = 0xFFFFFFFD，Thread/PSP/无FPU）
    LDMIA   R0!, {R4-R11, LR}

    ;; FPU 检查（初次启动时 LR=0xFFFFFFFD，bit4=1，不会执行）
    TST     LR, #0x10
    IT      EQ
    VLDMIAEQ R0!, {S16-S31}

    ;; 设置 PSP 指向硬件帧（BX LR 后硬件从此处弹出 R0-R3,R12,LR,PC,xPSR）
    MSR     PSP, R0
    ISB

    ;; 启动调度器时基（TIM6）
    ;; BL 会破坏 R0-R3 与 LR，故先把 LR（EXC_RETURN）压入 MSP 保护
    PUSH    {R0, LR}
    BL      scheduler_port_timer_start
    POP     {R0, LR}

    ;; 将 Thread 模式切换为使用 PSP（CONTROL bit1 = SPSEL = 1）
    MRS     R0, CONTROL
    ORR     R0, R0, #0x02
    MSR     CONTROL, R0
    ISB

    ;; 开中断，通过 EXC_RETURN 返回到第一个任务
    ;; LR = 0xFFFFFFFD → Thread 模式, PSP, 无 FPU 上下文
    CPSIE   I
    BX      LR

    ENDP


    END
