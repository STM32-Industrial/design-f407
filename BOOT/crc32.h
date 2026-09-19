#ifndef __CRC32_H
#define __CRC32_H
/* 标准 CRC-32 (IEEE 802.3, 多项式 0xEDB88320) */

#include <stdint.h>

uint32_t crc32(const uint8_t *buf, uint32_t len);

#endif /* __CRC32_H */
