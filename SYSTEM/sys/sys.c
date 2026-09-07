#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////

// THUMB 指令不支持汇编返回值
// 使用 CMSIS 内在函数实现(AC5/AC6 编译器均兼容)
// 进入 WFI 睡眠模式
void WFI_SET(void)
{
	__ASM volatile ("wfi");
}
// 关闭所有中断(但是不包括 fault 和 NMI 中断)
void INTX_DISABLE(void)
{
	__ASM volatile ("cpsid i");
}
// 开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile ("cpsie i");
}
// 设置栈顶地址
// addr:栈顶地址
void MSR_MSP(u32 addr)
{
	__ASM volatile ("msr msp, %0" :: "r" (addr));
}