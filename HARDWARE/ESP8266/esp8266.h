#ifndef __ESP8266_H
#define __ESP8266_H

#include "sys.h"
#include "usart.h"

// ESP-01S 通过 USART3(PB10=TX, PB11=RX) 与 STM32F407 通信
// 注意:ESP-01S 与 F407 之间的 RX/TX 要交叉连接!
//      F407 PB10(TX) -> ESP-01S RXD
//      F407 PB11(RX) -> ESP-01S TXD
//      共地, 3.3V 供电(ESP-01S 峰值电流大, 建议用独立3.3V电源)

#define ESP8266_RX_BUF_SIZE  512   // 接收缓冲区大小

extern volatile u8  ESP8266_RX_BUF[ESP8266_RX_BUF_SIZE];  // 接收缓冲区 (USART3中断写, 必须volatile)
extern volatile u16 ESP8266_RX_CNT;                       // 已接收字节数 (USART3中断写, 必须volatile)

void esp8266_init(u32 bound);                    // 初始化USART3,连接ESP-01S
void esp8266_send_string(char *s);               // 发送字符串(不追加任何内容)
void esp8266_send_cmd(char *cmd);                // 发送AT指令(自动追加\r\n)
void esp8266_clear_rxbuf(void);                  // 清空接收缓冲区
u8   esp8266_wait_string(char *str, u16 timeout_ms); // 在缓冲中等待指定字符串出现
void esp8266_print_rxbuf(void);                  // 将接收到的数据通过串口1打印

#endif
