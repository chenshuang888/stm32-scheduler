#ifndef __PORT_TIMER_H
#define __PORT_TIMER_H

/*===========================================================================
 * port_timer — 调度器时基移植层
 *
 * 职责：为调度器提供 1kHz 周期中断源（TIM6）。
 * 与 port_asm.s（CPU 上下文切换移植层）共同构成完整的移植层，
 * 换 MCU 时只需替换这一对文件，scheduler.c 无需改动。
 *
 * 设计要点：
 *   初始化与启动被拆成两个函数。TIM6 在 scheduler_init() 中完成配置并使能
 *   NVIC，但计数器保持停止（CEN=0）；真正的计数启动由 port_asm.s 的
 *   SVC_Handler 在切换到第一个任务前完成。
 *
 *   原因：scheduler_start() 之前 PSP 尚未初始化（Task 栈帧未生效），若此时
 *   产生时基中断，scheduler_tick() 会置位 PendSV，而 PendSV_Handler 会向
 *   PSP(=0) 执行 STMDB，必然触发 BusFault。推迟到 SVC 中启动可彻底消除
 *   这一窗口 —— SVC 优先级为 0，SysTick(15)/TIM6(15) 均无法抢占。
 *=========================================================================*/

/* 配置 TIM6 为 1kHz 时基并使能 NVIC，但不启动计数（CEN=0） */
void scheduler_port_timer_init(void);

/* 启动 TIM6 计数。只能由 port_asm.s 的 SVC_Handler 调用 */
void scheduler_port_timer_start(void);

#endif /* __PORT_TIMER_H */
