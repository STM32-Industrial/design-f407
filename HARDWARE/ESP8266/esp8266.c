#include "esp8266.h"
#include "delay.h"

// ESP-01S 驱动:基于 USART3(PB10=TX, PB11=RX),波特率115200
// 通过接收中断把ESP-01S返回的数据存入缓冲区,供上层检查关键字
// 注意: RX_BUF/RX_CNT 由 USART3 接收中断写入、任务中轮询读取, 必须 volatile (否则 -Oz 优化下读不到新数据)

volatile u8  ESP8266_RX_BUF[ESP8266_RX_BUF_SIZE];  // 接收缓冲区
volatile u16 ESP8266_RX_CNT = 0;                   // 已接收字节数

// 初始化USART3,用于与ESP-01S通信
void esp8266_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);      // 使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);     // 使能USART3时钟

    // PB10->USART3_TX, PB11->USART3_RX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;              // 波特率(ESP-01S默认115200)
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);            // 使能USART3
    USART_ClearFlag(USART3, USART_FLAG_TC);

    // 开启接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    esp8266_clear_rxbuf();
}

// USART3接收中断:把ESP-01S返回的数据存入缓冲区
void USART3_IRQHandler(void)
{
    u8 res;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART3);
        if (ESP8266_RX_CNT < ESP8266_RX_BUF_SIZE - 1)
        {
            ESP8266_RX_BUF[ESP8266_RX_CNT++] = res;
        }
    }
    /* 溢出错误(ORE): 擦除Flash等长时间关中断场景下RDR会溢出, ORE置位后
       若不读取SR+DR清除, 后续接收会永久停止(ESP数据全部收不到)。 */
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART3);   /* 读SR(GetFlagStatus)再读DR即清ORE */
    }
}

// 发送字符串(原样发送,不追加任何内容)
void esp8266_send_string(char *s)
{
    while (*s)
    {
        while ((USART3->SR & 0X40) == 0);   // 等待发送完成
        USART3->DR = (u8)(*s++);
    }
}

// 发送AT指令(自动追加回车换行)
void esp8266_send_cmd(char *cmd)
{
    esp8266_send_string(cmd);
    esp8266_send_string("\r\n");
}

// 清空接收缓冲区
void esp8266_clear_rxbuf(void)
{
    u16 i;
    for (i = 0; i < ESP8266_RX_BUF_SIZE; i++) ESP8266_RX_BUF[i] = 0;
    ESP8266_RX_CNT = 0;
}

// 缓冲中是否已有未处理的 MQTT 异步推送 (+MQTTSUBRECV)
// 用于让 mqtt_task 在接收推送期间暂停数据发布, 避免 clear_rxbuf 误清推送帧
u8 esp8266_has_pending_msg(void)
{
    static const char mark[] = "+MQTTSUBRECV:";
    u16 i, j;
    u16 cnt = ESP8266_RX_CNT;

    if (cnt < sizeof(mark) - 1) return 0;
    for (i = 0; i + sizeof(mark) - 1 <= cnt; i++)
    {
        for (j = 0; j < sizeof(mark) - 1; j++)
            if (ESP8266_RX_BUF[i + j] != (u8)mark[j]) break;
        if (j == sizeof(mark) - 1) return 1;
    }
    return 0;
}

// 在接收缓冲区中查找指定字符串
// 找到返回1,超时返回0
u8 esp8266_wait_string(char *str, u16 timeout_ms)
{
    u16 i, j = 0, timeout = 0;

    while (timeout < timeout_ms)
    {
        for (i = 0; i < ESP8266_RX_CNT; i++)   // 扫描整个缓冲
        {
            if (ESP8266_RX_BUF[i] == str[j])
            {
                j++;
                if (str[j] == '\0') return 1;  // 匹配成功
            }
            else
            {
                j = 0;
            }
        }
        delay_ms(1);                          // 等待更多数据
        timeout++;
    }
    return 0;
}

// 把接收缓冲区的数据通过串口1打印到电脑串口助手
void esp8266_print_rxbuf(void)
{
    u16 i;
    if (ESP8266_RX_CNT == 0)
    {
        printf("    (no data returned)\r\n");
        return;
    }
    printf("    RX: ");
    for (i = 0; i < ESP8266_RX_CNT; i++)
    {
        printf("%c", ESP8266_RX_BUF[i]);
    }
    printf("\r\n");
}
