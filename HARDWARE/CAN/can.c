#include "can.h"
#include "misc.h"
#include "stm32f4xx_can.h"

//////////////////////////////////////////////////////////////////////////////////
// CAN1 GPIO: PA11=RX, PA12=TX (正点原子探索者板载TJA1050, 需把P11跳线拨到CAN档)
// CAN1 时钟挂在 APB1 (42MHz)
// 500kbps: 42MHz / 500k = 84 个时钟/位;  (BS1+BS2+1)=14TQ, 预分频 = 84/14 = 6
//////////////////////////////////////////////////////////////////////////////////

// 环形缓冲与计数由 CAN1_FIFO0 接收中断写入、app任务读取, 必须 volatile (否则 -Oz 优化下收不到新帧)
volatile sensor_frame_t CAN_RX_BUF[CAN_RX_BUF_SIZE];
volatile u8  CAN_RX_WRITE = 0;
volatile u8  CAN_RX_READ  = 0;
volatile u8  CAN_RX_COUNT = 0;
volatile u32 CAN_RX_TOTAL = 0;

volatile machine_frame_t CAN_MACHINE_BUF[CAN_RX_BUF_SIZE];
volatile u8  CAN_MACHINE_WRITE = 0;
volatile u8  CAN_MACHINE_READ  = 0;
volatile u8  CAN_MACHINE_COUNT = 0;
volatile u32 CAN_MACHINE_TOTAL = 0;

// 计算校验和: 前 len 字节求和取低8位
u8 CAN_Checksum(const u8 *data, u8 len)
{
    u8 i;
    u8 sum = 0;
    for (i = 0; i < len; i++)
        sum += data[i];
    return sum;
}

// 初始化 CAN1 (500kbps, 正常模式, FIFO0 中断接收)
void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // 2. PA11=RX, PA12=TX 配置为复用推挽
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 引脚复用功能: CAN1 (AF9)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    // 4. CAN 工作模式: 500kbps
    CAN_DeInit(CAN1);
    CAN_InitStructure.CAN_TTCM = DISABLE;   // 时间触发关闭
    CAN_InitStructure.CAN_ABOM = ENABLE;    // 自动离线恢复
    CAN_InitStructure.CAN_AWUM = DISABLE;   // 自动唤醒关闭
    CAN_InitStructure.CAN_NART = DISABLE;   // 非自动重传 (出错自动重发)
    CAN_InitStructure.CAN_RFLM = DISABLE;   // FIFO 不锁定 (满则覆盖旧帧)
    CAN_InitStructure.CAN_TXFP = DISABLE;   // 发送优先级由 ID 决定
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = CAN_BS1_7tq;
    CAN_InitStructure.CAN_BS2  = CAN_BS2_6tq;
    CAN_InitStructure.CAN_Prescaler = 6;
    CAN_Init(CAN1, &CAN_InitStructure);

    // 5. 过滤器0: 只接收 ID=0x0001 的标准数据帧, 挂到 FIFO0
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_SENSOR_ID << 5) & 0xFFFF;  // STID -> 高16位
    CAN_FilterInitStructure.CAN_FilterIdLow  = 0x0000;                         // 标准帧: EXID=0, IDE=0, RTR=0
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;                     // 只比较11位标准ID
    CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;                     // IDE/RTR不比较
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);   // F4库此函数只有一个参数

    // 5.2 过滤器1: 只接收 ID=0x0002 机器参数帧, 同样挂 FIFO0
    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_MACHINE_ID << 5) & 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;                     // 只比较11位标准ID
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // 6. 使能 FIFO0 消息挂起中断
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    // 7. 配置 NVIC (抢占优先级1)
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 测试用: 发送一帧 ID=0x0001 测试数据 (自收发验证 F407 CAN 通路)
void CAN1_Send_Test(void)
{
    CanTxMsg TxMessage;
    u8 data[8];
    u8 i, sum = 0;

    data[0] = 12; data[1] = 34;   // 测试温度 12.34
    data[2] = 56; data[3] = 78;   // 测试湿度 56.78
    data[4] = 90; data[5] = 0;    // 测试烟雾 90.00
    data[6] = 0x03;               // 状态
    for (i = 0; i < 7; i++) sum += data[i];
    data[7] = sum;                // 校验和

    TxMessage.StdId = CAN_SENSOR_ID;
    TxMessage.ExtId = 0;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for (i = 0; i < 8; i++) TxMessage.Data[i] = data[i];

    CAN_Transmit(CAN1, &TxMessage);
}

// 取出缓冲区中的一帧 (主循环调用), 成功返回1, 无数据返回0
u8 CAN1_Receive_Frame(sensor_frame_t *frame)
{
    if (CAN_RX_COUNT == 0)
        return 0;

    *frame = CAN_RX_BUF[CAN_RX_READ];
    CAN_RX_READ = (CAN_RX_READ + 1) % CAN_RX_BUF_SIZE;
    CAN_RX_COUNT--;
    return 1;
}

// 取出机器参数缓冲区中的一帧 (主循环调用), 成功返回1, 无数据返回0
u8 CAN1_Receive_MachineFrame(machine_frame_t *frame)
{
    if (CAN_MACHINE_COUNT == 0)
        return 0;

    *frame = CAN_MACHINE_BUF[CAN_MACHINE_READ];
    CAN_MACHINE_READ = (CAN_MACHINE_READ + 1) % CAN_RX_BUF_SIZE;
    CAN_MACHINE_COUNT--;
    return 1;
}

// 发送阀门控制帧 (ID=0x0005, 标准帧, DLC=8 带校验和, 与F103协议一致)
// cmd: CAN_CTRL_VALVE_CLOSE / CAN_CTRL_VALVE_OPEN, 成功返回1
u8 CAN1_Send_CtrlValve(u8 cmd)
{
    CanTxMsg TxMessage;
    u8 data[8];
    u8 i, sum = 0;
    u8 mailbox;
    u16 timeout = 0;

    data[0] = cmd;                            // 命令: 0=关阀 1=开阀
    for (i = 1; i < 7; i++) data[i] = 0;      // 其余字节保留
    for (i = 0; i < 7; i++) sum += data[i];
    data[7] = sum;                            // 校验和

    TxMessage.StdId = CAN_CTRL_ID;
    TxMessage.ExtId = 0;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 8;
    for (i = 0; i < 8; i++) TxMessage.Data[i] = data[i];

    mailbox = CAN_Transmit(CAN1, &TxMessage);
    while (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Pending)
    {
        if (++timeout > 0x0FFF) return 0;     // 超时(无ACK)判失败
    }
    return (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok) ? 1 : 0;
}

// CAN1 FIFO0 接收中断: 收帧 -> 校验 -> 按ID分发入环形缓冲
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        // 排空FIFO0: F103每2s连发两帧(传感器帧+机器帧), 必须全取出防止丢帧
        while (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

            // 只处理本协议帧: 标准帧 + 数据长度够 + 校验通过
            if (RxMessage.IDE != CAN_Id_Standard || RxMessage.DLC < 8)
                continue;

            // 传感器数据帧 ID=0x0001
            if (RxMessage.StdId == CAN_SENSOR_ID &&
                CAN_Checksum(RxMessage.Data, 7) == RxMessage.Data[7])
            {
                if (CAN_RX_COUNT < CAN_RX_BUF_SIZE)   // 缓冲满则丢弃
                {
                    CAN_RX_BUF[CAN_RX_WRITE].temp_int  = RxMessage.Data[0];
                    CAN_RX_BUF[CAN_RX_WRITE].temp_dec  = RxMessage.Data[1];
                    CAN_RX_BUF[CAN_RX_WRITE].humi_int  = RxMessage.Data[2];
                    CAN_RX_BUF[CAN_RX_WRITE].humi_dec  = RxMessage.Data[3];
                    CAN_RX_BUF[CAN_RX_WRITE].smoke_int = RxMessage.Data[4];
                    CAN_RX_BUF[CAN_RX_WRITE].smoke_dec = RxMessage.Data[5];
                    CAN_RX_BUF[CAN_RX_WRITE].status    = RxMessage.Data[6];
                    CAN_RX_BUF[CAN_RX_WRITE].checksum  = RxMessage.Data[7];
                    CAN_RX_WRITE = (CAN_RX_WRITE + 1) % CAN_RX_BUF_SIZE;
                    CAN_RX_COUNT++;
                    CAN_RX_TOTAL++;
                }
            }
            // 机器参数帧 ID=0x0002
            else if (RxMessage.StdId == CAN_MACHINE_ID &&
                     CAN_Checksum(RxMessage.Data, 7) == RxMessage.Data[7])
            {
                if (CAN_MACHINE_COUNT < CAN_RX_BUF_SIZE)   // 缓冲满则丢弃
                {
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].spindle_rpm =
                        ((u16)RxMessage.Data[0] << 8) | RxMessage.Data[1];  // 转速大端2字节
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].pressure_int = RxMessage.Data[2];
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].pressure_dec = RxMessage.Data[3];
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].vib_int     = RxMessage.Data[4];
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].vib_dec     = RxMessage.Data[5];
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].state       = RxMessage.Data[6];
                    CAN_MACHINE_BUF[CAN_MACHINE_WRITE].checksum    = RxMessage.Data[7];
                    CAN_MACHINE_WRITE = (CAN_MACHINE_WRITE + 1) % CAN_RX_BUF_SIZE;
                    CAN_MACHINE_COUNT++;
                    CAN_MACHINE_TOTAL++;
                }
            }
        }
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
