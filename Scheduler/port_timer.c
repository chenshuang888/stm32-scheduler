#include "port_timer.h"
#include "scheduler.h"

/*===========================================================================
 * TIM6 句柄（本文件私有）
 * TIM6 挂载 APB1，与 DAC 共用中断向量 TIM6_DAC_IRQHandler
 *=========================================================================*/
static TIM_HandleTypeDef htim6;

/*---------------------------------------------------------------------------
 * 配置 TIM6 为 1kHz 周期中断
 *
 * STM32F4 定时器时钟规则（易错点）：
 *   APB1 预分频 == 1  →  TIMxCLK = PCLK1
 *   APB1 预分频 != 1  →  TIMxCLK = PCLK1 × 2
 *
 * 本工程 HCLK=84MHz、APB1=DIV2 → PCLK1=42MHz → TIM6CLK=84MHz。
 * HAL 的 HAL_TIM_Base_Init() 不会自动处理这个 ×2，必须自行换算；
 * 且此处采用动态计算而非硬编码，后续修改 PLL 或 APB1 分频均无需改动。
 *-------------------------------------------------------------------------*/
void scheduler_port_timer_init(void)
{
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)   /* PPRE1[2:0] != 000 即存在分频 */
        timclk *= 2U;                          /* → 定时器时钟 = PCLK1 × 2 */

    /* 必须在 HAL_TIM_Base_Init() 之前开时钟：后者会写 TIM6 寄存器 */
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = (timclk / 1000000U) - 1U;  /* 分频至 1MHz  */
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 1000U - 1U;                /* 1000 次溢出 → 1kHz */
    htim6.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    /* 与 PendSV 同为最低优先级 15：
       同级不嵌套，TIM6 返回后尾链进入 PendSV，行为与原 SysTick 版本一致 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 注意：此处不启动计数，CEN 保持 0。
       计数由 scheduler_port_timer_start() 在 SVC_Handler 中启动 */
}

/*---------------------------------------------------------------------------
 * 启动 TIM6 计数并使能更新中断
 * 仅由 port_asm.s 的 SVC_Handler 在切换到第一个任务前调用
 *-------------------------------------------------------------------------*/
void scheduler_port_timer_start(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/*---------------------------------------------------------------------------
 * TIM6 更新中断 —— 调度器时基，每 1ms 一次
 *
 * 直接判标志清中断，不经 HAL_TIM_IRQHandler → HAL_TIM_PeriodElapsedCallback
 * 分发链，可减少中断开销，也避免与其他 TIM 共用回调。
 *-------------------------------------------------------------------------*/
void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)    != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
        scheduler_tick();
    }
}
