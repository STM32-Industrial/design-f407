#ifndef __OTA_H
#define __OTA_H

/* ============================================================
 * App 端 OTA 升级 (固件由 MQTT 分块下行)
 *
 * 上位机 -> 设备  主题 ota/fw :
 *     "B,<size>,<crc32hex>,<version>"  开始 (size字节, CRC32, 版本号)
 *     "D,<seq>,<hex>"                  数据块 (每块128字节, seq从0开始按序)
 *     "E"                              结束 (设备校验CRC后写元数据并复位)
 *     "A"                              放弃
 * 设备 -> 上位机  主题 ota/ack :
 *     "R,BEGIN" / "R,OK,<seq>" / "R,DONE" / "R,FAIL,<crc32hex>" / "R,ERR,.."
 *
 * 采用"一块一确认"的停等协议: 设备回 R,OK,<seq> 后上位机才发下一块,
 * 收不到确认就重发同一块(设备对重复块会重新确认, 不会重复写Flash)。
 * 下载写入槽位B, 校验通过后写元数据 ready_flag 并复位, 由 Bootloader 安装。
 * ============================================================ */

#include "sys.h"
#include "ota_meta.h"

/* ---- MQTT 主题 (与上位机约定, 改动需同步上位机) ---- */
#define OTA_TOPIC       "ota/fw"
#define OTA_ACK_TOPIC   "ota/ack"

/* ---- 分块参数 ---- */
#define OTA_CHUNK_SIZE  128      /* 每块原始字节数 (hex编码后256字符) */

/* ---- 看门狗 (IWDG) 参数 ---- */
#define WDG_RELOAD_NORMAL  500   /* LSI 32kHz/256 = 125Hz -> 约4秒 */
#define WDG_RELOAD_WIDE    4095  /* 约32.7秒: 长擦除/App启动期间临时放宽 */

/* ---- App 接口 ---- */
void ota_init(void);        /* 启动时调用: 读元数据, 判断是否需要"提交"新固件 */
void ota_poll(void);        /* mqtt_task 循环调用: 取出完整MQTT消息并分发处理 */
void ota_commit_tick(void); /* wdg_task 每秒调用: 系统健康运行足够久则提交新固件 */
u8   ota_is_active(void);   /* 1=正在下载固件 (此时暂停普通MQTT上报) */

/* ---- 看门狗 (实现放在 ota.c: OTA长擦除需要临时放宽超时) ---- */
void wdg_init(void);        /* 使能IWDG, 超时4秒 (由 wdg_task 调用) */
void wdg_feed(void);        /* 喂狗 */
void wdg_wide(void);        /* 临时放宽超时, 用于长擦除/App启动 */
void wdg_normal(void);      /* 恢复正常超时 */

/* ---- 由 main.c 实现: 处理非OTA主题(阀门控制)的消息 ---- */
void app_on_mqtt_msg(const char *topic, const char *payload);

#endif /* __OTA_H */