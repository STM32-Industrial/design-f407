/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "lcd.h"
#include "delay.h"

/* 致命异常可视化: LED1(PF10)常亮 + 满屏专用颜色 (每种故障一种颜色, 一眼定位) */
/* HardFault=红 MemManage=紫红 BusFault=黄 UsageFault=青 */
#define FAULT_SHOW(color)  do { GPIO_ResetBits(GPIOF, GPIO_Pin_10); LCD_Clear(color); } while(0)


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* 红屏 + LED1 = HardFault; 串口打印故障寄存器 + 崩溃点(PC/LR) */
  uint32_t stacked_pc = 0, stacked_lr = 0;
  uint32_t *sp;
  __asm volatile ("TST   LR, #0x04      \n"
                  "ITE   EQ              \n"
                  "MRSEQ R0, MSP         \n"
                  "MRSNE R0, PSP         \n"
                  "MOV   %0, R0          \n"
                  : "=r" (sp) : : "r0", "memory", "cc");
  if (sp != (void *)0)   /* 栈帧: R0,R1,R2,R3,R12,LR,PC,xPSR */
  {
    stacked_lr = sp[5];
    stacked_pc = sp[6];
  }
  FAULT_SHOW(RED);
  printf("\r\n[FAULT] HardFault! HFSR=0x%08lX CFSR=0x%08lX\r\n",
         (unsigned long)SCB->HFSR, (unsigned long)SCB->CFSR);
  printf("[FAULT] PC=0x%08lX LR=0x%08lX (在 LED.map 里查这个PC地址即可定位崩溃函数)\r\n",
         (unsigned long)stacked_pc, (unsigned long)stacked_lr);
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* 紫红屏 + LED1 = MemManage */
  FAULT_SHOW(MAGENTA);
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* 黄屏 + LED1 = BusFault */
  FAULT_SHOW(YELLOW);
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* 青屏 + LED1 = UsageFault */
  FAULT_SHOW(CYAN);
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/* FreeRTOS 移植层通过 FreeRTOSConfig.h 宏映射提供 SVC_Handler,
   因此这里不再定义, 否则与 port.c 生成的函数重名报错
void SVC_Handler(void)
{
}
*/

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/* FreeRTOS 移植层提供 PendSV_Handler (上下文切换), 此处不再定义
void PendSV_Handler(void)
{
}
*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/* FreeRTOS 移植层提供 SysTick_Handler (节拍中断), 此处不再定义
void SysTick_Handler(void)
{
 
}
*/

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
