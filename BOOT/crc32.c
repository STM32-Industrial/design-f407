#include "crc32.h"

static uint32_t s_crc_table[256];
static uint8_t  s_table_ready = 0;

static void crc32_init_table(void)
{
    uint32_t i, k;
    for (i = 0; i < 256; i++) {
        uint32_t c = i;
        for (k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
        }
        s_crc_table[i] = c;
    }
    s_table_ready = 1;
}

uint32_t crc32(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    if (!s_table_ready) {
        crc32_init_table();
    }
    for (i = 0; i < len; i++) {
        crc = s_crc_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFUL;
}
