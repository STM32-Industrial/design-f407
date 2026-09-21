#include "boot_flash.h"
#include "stm32f4xx.h"

/* F407ZG 1MB Flash 扇区起始地址表 */
static const uint32_t s_sector_addr[12] = {
    0x08000000UL, 0x08004000UL, 0x08008000UL, 0x0800C000UL,
    0x08010000UL, 0x08020000UL, 0x08040000UL, 0x08060000UL,
    0x08080000UL, 0x080A0000UL, 0x080C0000UL, 0x080E0000UL
};

static uint32_t boot_flash_sector_of(uint32_t addr)
{
    uint32_t i;
    for (i = 0; i < 11; i++) {
        if (addr < s_sector_addr[i + 1]) return i;
    }
    return 11;
}

/* 必须先清掉上次操作的错误标志, 否则后续 FLASH_WaitForLastOperation 会直接返回错误 */
static void boot_flash_clear_flags(void)
{
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

void boot_flash_erase(uint32_t start, uint32_t end)
{
    uint32_t s0 = boot_flash_sector_of(start);
    uint32_t s1 = boot_flash_sector_of(end - 1);
    uint32_t s;
    FLASH_Unlock();
    boot_flash_clear_flags();
    for (s = s0; s <= s1; s++) {
        FLASH_EraseSector(s, VoltageRange_3);
    }
    FLASH_Lock();
}

void boot_flash_program_word(uint32_t addr, uint32_t value)
{
    FLASH_Unlock();
    boot_flash_clear_flags();
    FLASH_ProgramWord(addr, value);
    FLASH_Lock();
}

void boot_flash_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t i = 0;
    FLASH_Unlock();
    boot_flash_clear_flags();
    while (i + 4 <= len) {
        uint32_t w = ((uint32_t)data[i]) |
                     (((uint32_t)data[i + 1]) << 8) |
                     (((uint32_t)data[i + 2]) << 16) |
                     (((uint32_t)data[i + 3]) << 24);
        FLASH_ProgramWord(addr + i, w);
        i += 4;
    }
    if (i < len) {                       /* 尾部不足4字节, 高位补0xFF */
        uint32_t w = 0xFFFFFFFFUL, k;
        for (k = 0; i + k < len; k++) {
            w &= ~(((uint32_t)0xFF) << (8 * k));
            w |= ((uint32_t)data[i + k]) << (8 * k);
        }
        FLASH_ProgramWord(addr + i, w);
    }
    FLASH_Lock();
}

int boot_flash_copy_verify(uint32_t dst, uint32_t src, uint32_t len)
{
    uint32_t i = 0;
    FLASH_Unlock();
    boot_flash_clear_flags();
    while (i + 4 <= len) {
        uint32_t w = *(volatile uint32_t *)(src + i);
        FLASH_ProgramWord(dst + i, w);
        if (*(volatile uint32_t *)(dst + i) != w) {
            FLASH_Lock();
            return -1;
        }
        i += 4;
    }
    if (i < len) {                       /* 尾部逐字节处理, 避免非对齐读 */
        uint32_t w = 0xFFFFFFFFUL, k;
        for (k = 0; i + k < len; k++) {
            uint8_t b = *(volatile uint8_t *)(src + i + k);
            w &= ~(((uint32_t)0xFF) << (8 * k));
            w |= ((uint32_t)b) << (8 * k);
        }
        FLASH_ProgramWord(dst + i, w);
        if (*(volatile uint32_t *)(dst + i) != w) {
            FLASH_Lock();
            return -1;
        }
    }
    FLASH_Lock();
    return 0;
}
