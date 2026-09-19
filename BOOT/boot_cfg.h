#ifndef __BOOT_CFG_H
#define __BOOT_CFG_H
/* ============================================================
 * Bootloader 专用配置 (App 工程不需要此文件)
 * ============================================================ */

#include "ota_meta.h"

/* ---- 串口恢复模式参数 (USART1, PA9/PA10) ---- */
#define BOOT_UART_BAUD       115200UL
#define BOOT_RECV_TIMEOUT_MS 8000UL

#endif /* __BOOT_CFG_H */
