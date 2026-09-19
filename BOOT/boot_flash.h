#ifndef __BOOT_FLASH_H
#define __BOOT_FLASH_H
/* Flash 擦写/复制工具 (Bootloader 专用) */

#include <stdint.h>

void boot_flash_erase(uint32_t start, uint32_t end);        /* 擦除覆盖[start,end)的扇区 */
void boot_flash_program_word(uint32_t addr, uint32_t value);/* 写1个32位字 */
void boot_flash_program(uint32_t addr, const uint8_t *data, uint32_t len); /* 从RAM写数据 */
int  boot_flash_copy_verify(uint32_t dst, uint32_t src, uint32_t len);     /* 复制并逐字回读,成功返回0 */

#endif /* __BOOT_FLASH_H */
