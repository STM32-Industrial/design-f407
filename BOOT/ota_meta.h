#ifndef __OTA_META_H
#define __OTA_META_H
/* ============================================================
 * OTA 元数据/Flash分区 公共定义
 * 由 Bootloader 工程与 App 工程共同引用, 修改必须两边一致!
 * ============================================================ */

#include <stdint.h>

/* ---- Flash 分区布局 (F407ZG 1MB) ---- */
#define APP_ADDR            0x08004000UL   /* 槽位A: App 固定运行地址 */
#define APP_MAX_SIZE        0x0007C000UL   /* 槽位A 容量 496KB */
#define STAGING_ADDR        0x08080000UL   /* 槽位B: 新固件暂存起始 */
#define STAGING_MAX_SIZE    0x0007E000UL   /* 槽位B 可用 508KB (0x08080000~0x080FDFFF) */
#define META_ADDR           0x080FF000UL   /* OTA 元数据区 (扇区11末尾 4KB) */

#define META_MAGIC          0x4F544132UL   /* "OTA2" */

/* ---- 标志字取值 (Flash 只能单调编程 1->0, 故用"置0表示置位") ---- */
#define META_FLAG_SET       0x00000000UL   /* 已置位(已编程为0) */
#define META_FLAG_CLEARED   0xFFFFFFFFUL   /* 未置位(擦除态) */

/* 元数据结构: 所有字段只能从0xFFFFFFFF编程为某值, 复位需整扇区擦除 */
typedef struct {
    uint32_t magic;       /* 0x00: 必须等于 META_MAGIC */
    uint32_t size;        /* 0x04: 新固件字节数 */
    uint32_t crc32;       /* 0x08: 新固件 CRC32 */
    uint32_t app_version; /* 0x0C: 固件版本号 */
    uint32_t ready_flag;  /* 0x10: SET=新固件已下载到槽位B,待安装 */
    uint32_t running_flag;/* 0x14: SET=新固件已装入槽位A,试用中 */
    uint32_t commit_flag; /* 0x18: SET=新固件已确认运行正常 */
    uint32_t trial[3];    /* 0x1C: 试用失败标志, 每个SET代表失败1次 */
    uint32_t reserved;    /* 0x28 */
} ota_meta_t;

/* ---- App 向量表合法性范围 (栈顶指针应指向内部SRAM) ---- */
#define APP_SP_MIN          0x20000000UL
#define APP_SP_MAX          0x20020000UL

#endif /* __OTA_META_H */
