#include "touch.h"
#include "delay.h"
#include "lcd.h"
#include "stm32f4xx_gpio.h"
#include "stdio.h"

//////////////////////////////////////////////////////////////////////////////////
// 触摸屏底层: 软件SPI 读取 XPT2046 原始坐标, 并转换为屏幕像素坐标
// 采用"松手触发"(按下时采样并记录, 抬起时返回一次点击), 适合按键控制, 不会重复触发。
//////////////////////////////////////////////////////////////////////////////////

// 一次读取一个测量通道的 ADC 值(单次)
static u16 TP_ReadAD_One(u8 cmd)
{
    u8  i;
    u16 val = 0;

    TP_CS_RESET;            // 片选使能
    TP_SCK_RESET;

    // 发送 8 位命令字节 (XPT2046 DCLK上限2.5MHz, 每周期需>=400ns, 必须加延时)
    for (i = 0; i < 8; i++)
    {
        if (cmd & 0x80) TP_MOSI_SET; else TP_MOSI_RESET;
        delay_us(1);        // 数据建立
        TP_SCK_SET;
        delay_us(1);        // 高电平保持
        TP_SCK_RESET;
        delay_us(1);        // 低电平保持 (SCK完成一个完整周期)
        cmd <<= 1;
    }

    // 等待ADC转换完成 (BUSY结束; XPT2046转换时间约3~10us)
    delay_us(10);

    // 读取 12 位结果: BUSY释放后 DOUT 在第一个下降沿输出 DB11,
    // 之后每个下降沿输出下一位 (对齐正点原子官方时序, 无需补时钟)
    // 注意: 之前这里多补了一个时钟把 DB11 吃掉, 导致读数=(raw&0x7FF)*2,
    //       真实值跨2048回绕 -> 屏幕中点跳变 (已修复)
    for (i = 0; i < 12; i++)
    {
        TP_SCK_SET;
        delay_us(1);
        TP_SCK_RESET;
        delay_us(1);
        val <<= 1;
        if (TP_READ_MISO) val |= 1;
    }

    TP_CS_SET;              // 片选释放
    return val;
}

// 带多次采样平均的 ADC 读取, 抑制电阻屏抖动
u16 TP_ReadAD(u8 cmd)
{
    u32 sum = 0;
    u8  i;
    for (i = 0; i < 6; i++)
        sum += TP_ReadAD_One(cmd);
    return (u16)(sum / 6);
}

// 初始化触摸 IO
// 输出: T_CS(PC13), T_MOSI/DIN(PF11), T_SCK/CLK(PB0)
// 输入: T_MISO/DOUT(PB2), T_PEN(PB1), 加上拉
// 注意: 须在 LCD/FSMC 初始化之后调用
void TP_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOF, ENABLE);

    // 输出: T_CS(PC13)
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 输出: T_MOSI/DIN(PF11)
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    // 输出: T_SCK/CLK(PB0)
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 输入: T_MISO/DOUT(PB2), T_PEN(PB1), 上拉
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TP_CS_SET;              // 初始释放片选
    TP_SCK_RESET;           // 时钟空闲低
}

// ================= 坐标校准 (线性) =================
// 像素 = (raw - off) * scale / 1024
static long cal_x_off = TP_X_OFF;
static long cal_y_off = TP_Y_OFF;
static long cal_x_scale = TP_X_SCALE;
static long cal_y_scale = TP_Y_SCALE;

// 对 raw 套用方向校正 (TP_SWAP/TP_REV_X/TP_REV_Y)
static void tp_transform(u16 *rx, u16 *ry)
{
#if TP_SWAP
    { u16 t = *rx; *rx = *ry; *ry = t; }
#endif
#if TP_REV_X
    *rx = 4095 - *rx;
#endif
#if TP_REV_Y
    *ry = 4095 - *ry;
#endif
}

// raw 坐标 -> 屏幕像素坐标 (带校准系数)
static void tp_map(u16 rx, u16 ry, u16 *px, u16 *py)
{
    long vx = ((long)((long)rx - cal_x_off) * cal_x_scale) >> 10;
    long vy = ((long)((long)ry - cal_y_off) * cal_y_scale) >> 10;
#if TP_X_FLIP
    vx = (long)lcddev.width - 1 - vx;             // 左右(X)取反
#endif
    if (vx < 0) vx = 0; else if (vx >= (long)lcddev.width)  vx = (long)lcddev.width  - 1;
    if (vy < 0) vy = 0; else if (vy >= (long)lcddev.height) vy = (long)lcddev.height - 1;
    *px = (u16)vx;
    *py = (u16)vy;
}

// 实时触摸: 返回1=手指当前正按着屏幕, 并给出校准后像素坐标
u8 TP_Read(u16 *px, u16 *py)
{
    u16 rx, ry;
    if (TP_READ_PEN == TP_RELEASE) return 0;   // 没按
    delay_ms(2);                                // 极短稳定
    rx = TP_ReadAD(TP_X_CMD);
    ry = TP_ReadAD(TP_Y_CMD);
    tp_transform(&rx, &ry);
    tp_map(rx, ry, px, py);
    return 1;
}

// 画一个十字 (横条+竖条)
static void tp_draw_cross(u16 x, u16 y, u16 color)
{
    LCD_Fill(x - 8, y, x + 8, y, color);       // 横
    LCD_Fill(x, y - 8, x, y + 8, color);       // 竖
}

// 4点校准 (多次取平均): 依次按屏幕"左/右/顶/底"四个边缘中点
// 锚定左端->小px、右端->大px(顶/底同理), 无论面板方向都不会反. 结果打印到串口, 可固化到 touch.h
void TP_Calibrate(void)
{
    /* 十字锚点贴近屏幕边缘绘制(留10px边), 让标题栏按钮(y=6..30)和底部
     * 阀门按钮(y=276..320)进入校准覆盖区 (之前15/225,30/290时全部落在
     * 校准区外的外插区, 导致按不到/位置偏).
     * 注意: 下方线性计算仍用固定目标px(15/225,30/290) —— 手指按边缘十字时
     * 实际落点会比十字中心偏内约半指宽, 这种轻微外扩映射恰好补偿指压偏移,
     * 2026-09-05 实测三个按钮全部按准, 系数已固化到 touch.h */
    static const u16 tx[4] = { 10, 230, 120, 120 };   // 屏幕 px: 左中/右中/顶中/底中
    static const u16 ty[4] = { 160, 160, 10, 310 };
    static const char *tms[4] = { "LEFT", "RIGHT", "TOP", "BOTTOM" };
    long rvx[4] = {0,0,0,0}, rvy[4] = {0,0,0,0};      // 各点 raw 累加
    long rx0, rx1, ry0, ry1;
    u8   i, p, n;
    u32  sx, sy;

    LCD_Clear(BLUE);
    LCD_ShowString(15, 80, 210, 16, 16, "CALIBRATION");
    LCD_ShowString(15, 140, 210, 16, 16, "Touch 4 EDGES,");
    LCD_ShowString(15, 164, 210, 16, 16, "each few times");
    delay_ms(2500);

    for (i = 0; i < 4; i++)            // 4 个边缘点
    {
        char buf[20];
        LCD_Clear(BLUE);
        LCD_ShowString(15, 100, 210, 16, 16, "Touch & hold the");
        LCD_ShowString(15, 124, 210, 16, 16, "CROSS on the");
        sprintf(buf, "    %s edge", tms[i]);
        LCD_ShowString(15, 148, 210, 16, 16, buf);
        LCD_ShowString(15, 196, 210, 16, 16, "then release");
        for (p = 0; p < TP_CAL_PASSES; p++)      // 每个点多次采集
        {
            tp_draw_cross(tx[i], ty[i], YELLOW);

            while (TP_READ_PEN == TP_RELEASE) {}        // 等待按下
            delay_ms(80);
            sx = 0; sy = 0; n = 0;
            while (TP_READ_PEN == TP_PRESS)             // 按住采样
            {
                sx += TP_ReadAD(TP_X_CMD);
                sy += TP_ReadAD(TP_Y_CMD);
                n++;
                delay_ms(2);
                if (n >= 50) break;
            }
            if (n == 0) { sx = TP_ReadAD(TP_X_CMD); sy = TP_ReadAD(TP_Y_CMD); n = 1; }

            rvx[i] += sx / n;
            rvy[i] += sy / n;

            while (TP_READ_PEN == TP_PRESS) {}          // 松开再进入同点下一次
            tp_draw_cross(tx[i], ty[i], BLUE);          // 擦十字
            delay_ms(250);
        }
    }

    // 各点取多次平均
    for (i = 0; i < 4; i++) { rvx[i] /= TP_CAL_PASSES; rvy[i] /= TP_CAL_PASSES; }

    // X 方向: 左中点(px=15) vs 右中点(px=225), 跨度210
    rx0 = rvx[0];  rx1 = rvx[1];
    // Y 方向: 顶中点(px=30)  vs 底中点(px=290), 跨度260
    ry0 = rvy[2];  ry1 = rvy[3];

    if (rx1 != rx0)
    {
        cal_x_scale = (210L * 1024L) / (rx1 - rx0);
        cal_x_off   = rx0 - (15L * 1024L) / cal_x_scale;
    }
    if (ry1 != ry0)
    {
        cal_y_scale = (260L * 1024L) / (ry1 - ry0);
        cal_y_off   = ry0 - (30L * 1024L) / cal_y_scale;
    }

    printf("[CALIB] x_off=%ld x_scale=%ld y_off=%ld y_scale=%ld (raw L=%ld R=%ld T=%ld B=%ld)\r\n",
           cal_x_off, cal_x_scale, cal_y_off, cal_y_scale, rx0, rx1, ry0, ry1);

    LCD_Clear(WHITE);
}