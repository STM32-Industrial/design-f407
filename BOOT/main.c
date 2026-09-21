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

/* 分频统一用256, 且超时给足: App 初始化(LCD+LVGL+界面)需要1~3秒,
 * 若这里只给3秒, 跳转后 App 还没跑完就被复位(黑屏/无输出)。
 * App 起来后由 wdg_task 把重载值改回 500(约4秒)进入正常监督。
 * 注意: 分频必须由这里统一设定, App 侧不再改分频。 */
#define BOOT_IWDG_PRESCALER  IWDG_Prescaler_256  /* LSI~32kHz/256 = 125Hz */
#define BOOT_IWDG_RELOAD     4095UL              /* 约32.8秒超时 */
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

    /* 跳转前把 BOOT 用过的外设关掉并复位, 否则 App 会继承不正常的外设状态
       (实测: USART1 停在发送中会让 App 的 printf 死等 TC 标志, 导致黑屏+无输出) */
    boot_uart_deinit();

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
    uint32_t guard;

    /* 关键: IWDG 的 PR/RLR 写入后需要等 PVU/RVU 标志清零才算真正生效
     * (需几个 LSI 周期同步)。若写完立刻 Enable, 分频会停在复位默认值 4 分频,
     * 超时就从 32 秒变成 4095*4/32000 ≈ 0.5 秒 —— App 还没跑完就被反复复位,
     * 表现就是屏幕黑屏 + App 没有任何串口输出。
     * 这里带 guard 计数等待, 避免 LSI 异常时死循环。 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(BOOT_IWDG_PRESCALER);
    for (guard = 0; guard < 2000000UL && (IWDG->SR & IWDG_SR_PVU); guard++) { }
    IWDG_SetReload(BOOT_IWDG_RELOAD);
    for (guard = 0; guard < 2000000UL && (IWDG->SR & IWDG_SR_RVU); guard++) { }
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
    /* 安装完成: 擦除元数据扇区可同时清掉 ready_flag(并丢弃槽位B暂存数据),
       再写回 magic/版本/running_flag, 这样下次启动才会走"试用计数"逻辑,
       否则每次启动都会重装一遍, 试用失败回滚就永远触发不了。 */
    boot_meta_erase();
    boot_meta_write_field(offsetof(ota_meta_t, magic), META_MAGIC);
    boot_meta_write_field(offsetof(ota_meta_t, app_version), s_meta.app_version);
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
    /* 自检: 打印 IWDG 实际生效的分频与重载值, 便于确认配置是否真的写进去了。
     * 正常应为 PR=4(256分频) RLR=00000FFF(约32.8秒)。 */
    boot_uart_puts("[BOOT] IWDG PR=");
    boot_uart_puthex32(IWDG->PR);
    boot_uart_puts(" RLR=");
    boot_uart_puthex32(IWDG->RLR);
    boot_uart_puts("\r\n");
    boot_uart_puts("[BOOT] 启动App @0x08004000\r\n");
    boot_jump_app();

    return 0;
}
