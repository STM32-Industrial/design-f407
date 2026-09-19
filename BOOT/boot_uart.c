#include "boot_uart.h"
#include "stm32f4xx.h"

void boot_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

void boot_delay_ms(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 168000UL; j++);   /* 168MHz 下约1ms */
    }
}

void boot_uart_putc(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)c);
}

void boot_uart_puts(const char *s)
{
    while (*s) {
        boot_uart_putc(*s++);
    }
}

void boot_uart_puthex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t i;
    boot_uart_puts("0x");
    for (i = 0; i < 8; i++) {
        boot_uart_putc(hex[(v >> (28 - 4 * i)) & 0xF]);
    }
}

/* 接收n字节; 快速轮询RXNE避免溢出; 超时返回-1 */
int boot_uart_receive(uint8_t *buf, uint32_t n, uint32_t timeout_ms)
{
    uint32_t got = 0;
    while (got < n) {
        uint32_t spins = 0;
        uint32_t deadline = timeout_ms;
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {
            if (++spins > 500000UL) {    /* 约数ms @168MHz */
                if (deadline == 0) return -1;
                deadline--;
                spins = 0;
            }
        }
        buf[got++] = (uint8_t)USART_ReceiveData(USART1);
    }
    return 0;
}

int boot_uart_readline(char *buf, uint32_t maxlen, uint32_t timeout_ms)
{
    uint32_t i = 0;
    while (i < maxlen - 1) {
        uint8_t b;
        if (boot_uart_receive(&b, 1, timeout_ms) != 0) return -1;
        if (b == '\r' || b == '\n') {
            if (i == 0) continue;        /* 跳过前导换行 */
            break;
        }
        buf[i++] = (char)b;
    }
    buf[i] = '\0';
    return (int)i;
}
