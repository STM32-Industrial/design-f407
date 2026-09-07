#include "delay.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
// 延时函数 (FreeRTOS 版本)
// - SysTick 时钟 = HCLK (168MHz), 与 FreeRTOS 内核保持一致
// - 调度器启动后, delay_ms 使用 vTaskDelay 让出 CPU (不再阻塞其它任务)
// - delay_us 保持忙等待 (不修改 SysTick->LOAD, 与 FreeRTOS 共存)
//////////////////////////////////////////////////////////////////////////////////

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#include "task.h"
#endif

static u8  fac_us = 0;   // us 延时倍乘数 (HCLK MHz 数)
static u16 fac_ms = 0;   // ms 延时倍乘数, OS 下 = 1000/configTICK_RATE_HZ

// 初始化延时函数
// SYSCLK: 系统时钟 (F407 为 168)
void delay_init(u8 SYSCLK)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  // SysTick = HCLK (与FreeRTOS一致)
    fac_us = SYSCLK;                                  // 每 us 计数 SYSCLK 次
#if SYSTEM_SUPPORT_OS
    fac_ms = 1000 / configTICK_RATE_HZ;               // tick=1000Hz 时 =1
#else
    fac_ms = (u16)fac_us * 1000;
#endif
    // 只启动计数(不开中断): 让调度器启动前 delay_us/delay_ms 可用;
    // 正式节拍中断由 FreeRTOS 在 vTaskStartScheduler() 里接管配置
    SysTick->LOAD = 0xFFFFFF;                         // 预装载大值, 计数器持续递减
    SysTick->VAL  = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// 忙等待延时 nus (通过读取计数寄存器递减来计时, 不改 SysTick->LOAD)
// 注意: 被更高优先级任务抢占时实际耗时可能略长, 属正常
void delay_us(u32 nus)
{
    u32 ticks;
    u32 told, tnow, tcnt = 0;
    u32 reload = SysTick->LOAD;

    ticks = nus * fac_us;
    told  = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told) tcnt += told - tnow;
            else             tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

// 延时 nms
// 调度器运行期间用 vTaskDelay 让出 CPU (不阻塞其它任务), 未启动前忙等待
void delay_ms(u16 nms)
{
#if SYSTEM_SUPPORT_OS
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        if (nms >= fac_ms)
            vTaskDelay(nms / fac_ms);   // 整 tick 部分交给系统
        nms %= fac_ms;
    }
#endif
    if (nms) delay_us((u32)(nms * 1000));
}
