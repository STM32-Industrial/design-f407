#ifndef __TOUCH_H
#define __TOUCH_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// XPT2046 电阻触摸屏驱动 (软件SPI)
//
// 实际接线 (用户提供, 当前为"MISO/MOSI对调"排查版):
//   T_CS    PC13  片选 (输出, 低有效)
//   T_SCK   PF0   时钟 (输出)
//   T_MISO  PF11  数据出 (输入)  // 触摸芯片 DOUT -> MCU   注: 怀疑与T_MOSI标反, 当前对调测试
//   T_MOSI  PB2   数据入 (输出)  // MCU -> 触摸芯片 DIN
//   T_PEN   PB1   按下中断检测 (输入, 按下为低)
//
// 兼容性说明:
//   触摸层的 ADC 坐标方向与屏幕显示方向不一定一致, 若点按反映的坐标反向/互换,
//   调整下方 TP_REV_X / TP_REV_Y / TP_SWAP 三个宏即可, 无需改其他代码。
//////////////////////////////////////////////////////////////////////////////////

// ---- ADC 原始坐标 -> 屏幕坐标 方向校正 ----
#define TP_REV_X   0   // X方向取反
#define TP_REV_Y   0   // Y方向取反 (若上下相反置1)
#define TP_SWAP    0   // X/Y互换 (若横竖颠倒置1)

// ---- 引脚操作 (软件SPI) ----
//   依据模块接口表(#29~#34):
//     #29 T_MISO -> PB2  (DOUT, 输入)
//     #30 MOSI   -> PF11 (DIN,  输出)
//     #31 T_PEN  -> PB1  (按下检测, 输入, 低有效)
//     #33 T_CS   -> PC13 (片选, 输出, 低有效)
//     #34 CLK    -> PB0  (SCK,  输出)   // 注意: 非PF0!
#define TP_CS_SET      GPIO_SetBits(GPIOC, GPIO_Pin_13)      // 片选 高=释放
#define TP_CS_RESET    GPIO_ResetBits(GPIOC, GPIO_Pin_13)    // 片选 低=选中
#define TP_SCK_SET     GPIO_SetBits(GPIOB, GPIO_Pin_0)       // SCK
#define TP_SCK_RESET   GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define TP_MOSI_SET    GPIO_SetBits(GPIOF, GPIO_Pin_11)      // DIN
#define TP_MOSI_RESET  GPIO_ResetBits(GPIOF, GPIO_Pin_11)
#define TP_READ_MISO   ((u8)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2))  // DOUT 输入
#define TP_READ_PEN    ((u8)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))  // 低=按下

// ---- XPT2046 命令 (差分12位) ----
#define TP_X_CMD   0xD0    // 测量 X 坐标
#define TP_Y_CMD   0x90    // 测量 Y 坐标

#define TP_PRESS    0      // PEN 为低, 已按下
#define TP_RELEASE  1      // PEN 为高, 未按下

// ---- 坐标校准 (线性): 像素 = (raw - OFF) * SCALE / 1024 ----
// 2026-09-05 贴边锚点(十字 x=10/230, y=10/310)重新校准实测固化:
//   X: off=369  scale=73   (raw L=579->15px, R=3487->222px)
//   Y: off=142   scale=89   (raw T=487->30px, B=3448->287px)
// 实测: 标题栏 TREND/返回 与 底部阀门 三个按钮全部按准 (线性计算仍用
// 固定常数15/225,30/290做目标px, 与贴边十字的轻微外扩映射补偿了指压偏移)
#define TP_X_OFF     369
#define TP_Y_OFF     142
#define TP_X_SCALE   73
#define TP_Y_SCALE   89

#define TP_DO_CALIB  0    // 0 = 跳过上电校准, 直接进 LVGL 界面 (如需重新校准临时改1)
#define TP_X_FLIP    0    // 校准会自动对准左右方向, 无需手动翻转 (先保持0)

#define TP_CAL_PASSES  3   // (未启用) 每个校准点重复采集次数

void  TP_Init(void);              // 初始化触摸 IO (软件SPI)
u16   TP_ReadAD(u8 cmd);          // 读一次 ADC 原始值 (含多次采样, 取平均)
u8    TP_Read(u16 *px, u16 *py);  // 实时触摸: 返回1=手指当前正按着屏幕, 并给出校准后像素坐标
void  TP_Calibrate(void);         // 4点校准: 依次点屏幕上4个十字, 计算并打印坐标系数

#endif