/* ============================================================
 * STM32F407 OTA Bootloader V1.0
 * 链接地址: 0x08000000 (扇区0, 16KB)
 * 功能: 引导App / 双槽复制安装 / 看门狗试用回滚 / 串口恢复
 * 串口: USART1 (PA9/PA10, 115200)
 * ============================================================ */

#include <stddef.h>
#include "ota_meta.h"
#include "boot_cfg.h"
#include "boot_flash.h"
#include "boot_uart.h"
#include "crc32.h"
#include "stm32f4xx.h"

static ota_meta_t s_meta;

#define BOOT_IWDG_PRESCALER  IWDG_Prescaler_64   /* LSI~32kHz/64 = 512Hz */
#define BOOT_IWDG_RELOAD     1500UL              /* 约3秒超时 */
#define BOOT_MAX_TRIALS      3                   /* 新固件试用失败上限 */

/* ---------------- 元数据 ---------------- */

static void boot_meta_read(void)
{
    const uint32_t *p = (const uint32_t *)META_ADDR;
    uint32_t *d = (uint32_t *)&s_meta;
    uint32_t i;
    for (i = 0; i < sizeof(ota_meta_t) / 4; i++) {
        d[i] = p[i];
    }
}

static void boot_meta_write_field(uint32_t offset, uint32_t value)
{
    boot_flash_program_word(META_ADDR + offset, value);
}

static void boot_meta_erase(void)
{
    boot_flash_erase(META_ADDR, META_ADDR + 4096);   /* 擦除扇区11, 同时丢弃残留暂存数据 */
}

/* ---------------- App 合法性检查 / 跳转 ---------------- */

static int boot_app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_ADDR;
    uint32_t pc = *(volatile uint32_t *)(APP_ADDR + 4);
    if (sp < APP_SP_MIN || sp >= APP_SP_MAX) return 0;
    if (pc < APP_ADDR || pc >= (APP_ADDR + APP_MAX_SIZE)) return 0;
    return 1;
}

static void boot_jump_app(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_ADDR;
    uint32_t pc = *(volatile uint32_t *)(APP_ADDR + 4);
    void (*app_entry)(void) = (void (*)(void))pc;

    __disable_irq();
    __set_MSP(sp);
    __set_CONTROL(0);
    __ISB();
    SCB->VTOR = APP_ADDR;
    __DSB();
    __ISB();
    app_entry();
    while (1) { }
}

static void boot_iwdg_start(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(BOOT_IWDG_PRESCALER);
    IWDG_SetReload(BOOT_IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ---------------- 串口恢复模式 ---------------- */
/* 协议: 收到 "U:<size_dec>:<crc32_hex>" 后, 按512B一块接收数据,
 *       每块编程完成后回 'K', PC端等 'K' 再发下一块. 收完校验CRC后跳转. */

static int boot_parse_update_header(const char *line, uint32_t *size, uint32_t *crc)
{
    const char *p;
    uint32_t s = 0, c = 0;
    int i;
    if (line[0] != 'U' || line[1] != ':') return 0;
    p = line + 2;
    while (*p >= '0' && *p <= '9') {
        s = s * 10 + (uint32_t)(*p - '0');
        p++;
    }
    if (*p != ':' || s == 0 || s > APP_MAX_SIZE) return 0;
    p++;
    for (i = 0; i < 8; i++) {
        uint32_t v;
        char ch = p[i];
        if (ch >= '0' && ch <= '9')      v = (uint32_t)(ch - '0');
        else if (ch >= 'A' && ch <= 'F') v = (uint32_t)(ch - 'A' + 10);
        else if (ch >= 'a' && ch <= 'f') v = (uint32_t)(ch - 'a' + 10);
        else return 0;
        c = (c << 4) | v;
    }
    *size = s;
    *crc = c;
    return 1;
}

static int boot_receive_and_program(uint32_t size)
{
    uint8_t block[512];
    uint32_t done = 0;
    while (done < size) {
        uint32_t chunk = (size - done >= 512) ? 512 : (size - done);
        if (boot_uart_receive(block, chunk, 8000) != 0) return -1;
        boot_flash_program(APP_ADDR + done, block, chunk);
        done += chunk;
        boot_uart_putc('K');
    }
    return 0;
}

static void boot_recovery_loop(void)
{
    char line[64];
    boot_uart_puts("\r\n[BOOT] ====== 串口恢复模式 ======\r\n");
    boot_uart_puts("[BOOT] 等待固件... 格式: U:<size>:<crc32hex>\r\n");
    while (1) {
        uint32_t size = 0, crc = 0;
        if (boot_uart_readline(line, sizeof(line), BOOT_RECV_TIMEOUT_MS) <= 0) {
            boot_uart_puts("[BOOT] 等待超时,继续等待...\r\n");
            continue;
        }
        if (!boot_parse_update_header(line, &size, &crc)) {
            boot_uart_puts("[BOOT] 格式错误,需 U:<size>:<crc32hex>\r\n");
            continue;
        }
        boot_uart_puts("[BOOT] 擦除槽位A...\r\n");
        boot_flash_erase(APP_ADDR, APP_ADDR + APP_MAX_SIZE);
        boot_uart_puts("[BOOT] 接收固件(每块512B,等K再发下一块)...\r\n");
        if (boot_receive_and_program(size)) {
            boot_uart_puts("[BOOT] 接收失败!\r\n");
            continue;
        }
        if (crc32((const uint8_t *)APP_ADDR, size) == crc) {
            boot_uart_puts("[BOOT] 校验通过,启动App!\r\n");
            boot_meta_erase();
            boot_iwdg_start();
            boot_jump_app();
        } else {
            boot_uart_puts("[BOOT] CRC不匹配!\r\n");
        }
    }
}

/* ---------------- OTA 安装 / 试用逻辑 ---------------- */

static void boot_install_from_staging(void)
{
    if (s_meta.size == 0 || s_meta.size > APP_MAX_SIZE) {
        boot_uart_puts("[BOOT] 尺寸非法,丢弃新固件\r\n");
        boot_meta_erase();
        return;
    }
    boot_uart_puts("[BOOT] 校验槽位B CRC...\r\n");
    if (crc32((const uint8_t *)STAGING_ADDR, s_meta.size) != s_meta.crc32) {
        boot_uart_puts("[BOOT] CRC失败,丢弃新固件\r\n");
        boot_meta_erase();
        return;
    }
    boot_uart_puts("[BOOT] CRC通过,擦除槽位A...\r\n");
    boot_flash_erase(APP_ADDR, APP_ADDR + APP_MAX_SIZE);
    boot_uart_puts("[BOOT] 复制 槽位B -> 槽位A ...\r\n");
    if (boot_flash_copy_verify(APP_ADDR, STAGING_ADDR, s_meta.size) != 0) {
        boot_uart_puts("[BOOT] 复制校验失败!\r\n");
        boot_meta_erase();
        return;
    }
    boot_meta_write_field(offsetof(ota_meta_t, running_flag), META_FLAG_SET);
    boot_uart_puts("[BOOT] 安装完成,进入试用\r\n");
}

static void boot_trial_logic(void)
{
    uint32_t trials = 0, i;
    for (i = 0; i < 3; i++) {
        if (s_meta.trial[i] == META_FLAG_SET) trials++;
    }
    boot_uart_puts("[BOOT] 试用失败次数: ");
    boot_uart_puthex32(trials);
    boot_uart_puts("\r\n");
    if (trials < BOOT_MAX_TRIALS) {
        boot_meta_write_field(offsetof(ota_meta_t, trial[0]) + trials * 4, META_FLAG_SET);
        boot_uart_puts("[BOOT] 再次启动App试用\r\n");
    } else {
        boot_uart_puts("[BOOT] 新固件多次启动失败,进入串口恢复模式\r\n");
        boot_recovery_loop();
    }
}

/* ---------------- 主流程 ---------------- */

int main(void)
{
    /* SystemInit 已由 startup 调用, 主频 168MHz */
    boot_uart_init(BOOT_UART_BAUD);
    boot_uart_puts("\r\n[BOOT] F407 OTA Bootloader V1.0\r\n");

    boot_meta_read();

    if (s_meta.magic == META_MAGIC) {
        if (s_meta.commit_flag == META_FLAG_SET) {
            boot_uart_puts("[BOOT] 状态: 新固件已确认(正常)\r\n");
        } else if (s_meta.ready_flag == META_FLAG_SET) {
            boot_uart_puts("[BOOT] 状态: 新固件就绪,开始安装...\r\n");
            boot_install_from_staging();
        } else {
            boot_uart_puts("[BOOT] 状态: 试用中\r\n");
            boot_trial_logic();
        }
    } else {
        boot_uart_puts("[BOOT] 无元数据(首次),直接启动App\r\n");
    }

    if (!boot_app_valid()) {
        boot_uart_puts("[BOOT] 槽位A无有效App,进入串口恢复模式\r\n");
        boot_recovery_loop();
    }

    boot_iwdg_start();
    boot_uart_puts("[BOOT] 启动App @0x08004000\r\n");
    boot_jump_app();

    return 0;
}
