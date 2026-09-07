#ifndef __CAN_H
#define __CAN_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// CAN 通信驱动  (STM32F407 探索者 + TJA1050 收发器)
//
// 引脚:  PA11 = CAN1_RX,  PA12 = CAN1_TX   (探索者板载TJA1050, P11跳线拨到CAN档)
// 波特率: 500kbps  (APB1=42MHz, 预分频=6, BS1=7TQ, BS2=6TQ)
// 模式:   CAN1 正常模式, FIFO0 中断接收
//
// 数据帧协议 (F103 传感器节点 -> F407 网关, 标准帧, ID=0x0001, DLC=8):
//   Data[0]  温度整数
//   Data[1]  温度小数 (0~99, 实际值 = 整数 + 小数/100)
//   Data[2]  湿度整数
//   Data[3]  湿度小数 (0~99)
//   Data[4]  烟雾浓度整数
//   Data[5]  烟雾浓度小数 (0~99)
//   Data[6]  状态字 (bit0: DHT11有效, bit1: MQ-2有效)
//   Data[7]  校验和 = Data[0]~Data[6] 求和取低8位
//////////////////////////////////////////////////////////////////////////////////

#define CAN_SENSOR_ID   0x0001     // 传感器数据帧 CAN ID (F103 -> F407)
#define CAN_MACHINE_ID  0x0002     // 机器参数帧 CAN ID (F103 -> F407)
#define CAN_CTRL_ID     0x0005     // 控制帧 CAN ID (F407 -> F103, 阀门控制)

// 控制帧命令定义 (Data[0])
#define CAN_CTRL_VALVE_CLOSE   0   // 关闭阀门
#define CAN_CTRL_VALVE_OPEN    1   // 打开阀门

#define CAN_RX_BUF_SIZE 32         // 接收环形缓冲大小

// 传感器数据帧结构 (与 CAN 数据域一一对应)
typedef struct
{
    u8 temp_int;      // 温度整数
    u8 temp_dec;      // 温度小数
    u8 humi_int;      // 湿度整数
    u8 humi_dec;      // 湿度小数
    u8 smoke_int;     // 烟雾浓度整数
    u8 smoke_dec;     // 烟雾浓度小数
    u8 status;        // 状态字
    u8 checksum;      // 校验和
} sensor_frame_t;

// 机器参数帧结构 (ID=0x0002, 与F103 CAN_SendMachineData 格式一致)
typedef struct
{
    u16 spindle_rpm;      // 主轴转速 rpm (大端2字节: Data[0]=高, Data[1]=低)
    u8  pressure_int;     // 压力整数 (MPa)
    u8  pressure_dec;     // 压力小数
    u8  vib_int;          // 振动整数 (mm/s)
    u8  vib_dec;          // 振动小数
    u8  state;            // 运行状态 0=停止 1=运行 2=空闲
    u8  checksum;         // 校验和
} machine_frame_t;

extern volatile sensor_frame_t CAN_RX_BUF[CAN_RX_BUF_SIZE];  // 环形缓冲 (中断写, 必须volatile)
extern volatile u8  CAN_RX_WRITE;    // 写索引 (中断中更新)
extern volatile u8  CAN_RX_READ;     // 读索引 (主循环更新)
extern volatile u8  CAN_RX_COUNT;    // 缓冲区中待处理帧数
extern volatile u32 CAN_RX_TOTAL;    // 成功接收的总帧数

extern volatile machine_frame_t CAN_MACHINE_BUF[CAN_RX_BUF_SIZE];  // 机器参数环形缓冲
extern volatile u8  CAN_MACHINE_WRITE;    // 写索引 (中断中更新)
extern volatile u8  CAN_MACHINE_READ;     // 读索引 (主循环更新)
extern volatile u8  CAN_MACHINE_COUNT;    // 缓冲区中待处理帧数
extern volatile u32 CAN_MACHINE_TOTAL;    // 成功接收的机器参数总帧数

void CAN1_Init(void);        // 初始化 CAN1 (500kbps, FIFO0 中断接收)
u8   CAN1_Receive_Frame(sensor_frame_t *frame);  // 取出一帧, 成功返回1
u8   CAN1_Receive_MachineFrame(machine_frame_t *frame);  // 取出一帧机器参数, 成功返回1
u8   CAN1_Send_CtrlValve(u8 cmd);   // 发送阀门控制帧 (ID=0x0005) 到 F103
u8   CAN_Checksum(const u8 *data, u8 len);       // 计算校验和 (前len字节求和)
void CAN1_Send_Test(void);   // 测试用: 发送一帧 ID=0x0001 测试数据 (自收发验证)

#endif
