#ifndef __BOOT_UART_H
#define __BOOT_UART_H
/* USART1 日志输出 + 串口恢复模式 接收 */

#include <stdint.h>

void boot_uart_init(uint32_t baud);
void boot_uart_putc(char c);
void boot_uart_puts(const char *s);
void boot_uart_puthex32(uint32_t v);
int  boot_uart_readline(char *buf, uint32_t maxlen, uint32_t timeout_ms);
int  boot_uart_receive(uint8_t *buf, uint32_t n, uint32_t timeout_ms);
void boot_delay_ms(uint32_t ms);

#endif /* __BOOT_UART_H */
