#include "ota.h"
#include "crc32.h"
#include "esp8266.h"
#include "delay.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "stddef.h"

/* ============================================================
 * App 端 OTA 接收
 *   1. 上位机通过 MQTT 主题 ota/fw 下发 "开始/数据块/结束"
 *   2. 数据块写入 槽位B (STAGING_ADDR), 每块回 ota/ack 确认
 *   3. 收完做 CRC32 校验, 通过则写元数据 ready_flag=SET 并复位
 *   4. Bootloader 检测到 ready_flag 后把槽位B 装入槽位A
 * 注意: 本文件的 Flash 操作会阻塞CPU数秒(扇区擦除), 期间用 wdg_wide()
 *       临时放宽看门狗, 否则会被 IWDG 复位。
 * ============================================================ */

/* F407ZG 1MB Flash 扇区起始地址表 */
static const u32 s_sector_addr[12] = {
    0x08000000UL, 0x08004000UL, 0x08008000UL, 0x0800C000UL,
    0x08010000UL, 0x08020000UL, 0x08040000UL, 0x08060000UL,
    0x08080000UL, 0x080A0000UL, 0x080C0000UL, 0x080E0000UL
};

static u8    s_active;        /* 1=正在接收数据块 */
static u32   s_size;          /* 本次固件总字节数 */
static u32   s_crc;           /* 上位机给的 CRC32 */
static u32   s_ver;           /* 版本号 */
static u32   s_seq;           /* 期望的下一块序号 */

static u8    s_commit_pending;/* 1=新固件已装好, 待确认提交 */
static u16   s_commit_cnt;    /* 健康运行秒数计数 */

static u8    s_chunk[OTA_CHUNK_SIZE];

/* ==================== 看门狗 ==================== */

/* 写 IWDG 重载值: 必须等 RVU 标志清零才真正生效(需几个LSI周期同步)。
 * 标准库不等, 直接连写会写不进去, 导致超时用回旧值。 */
static void iwdg_set_reload(u32 reload)
{
    u32 guard;
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetReload(reload);
    for (guard = 0; guard < 2000000UL && (IWDG->SR & IWDG_SR_RVU); guard++) { }
    IWDG_ReloadCounter();
}

void wdg_init(void)
{
    /* 分频固定 256(约125Hz), 由 Bootloader 在跳转前统一设定, 这里不再改分频。
       本函数只把超时从启动期的宽值收紧到正常监督值(约4秒)。 */
    iwdg_set_reload(WDG_RELOAD_NORMAL);
}

void wdg_feed(void)
{
    IWDG_ReloadCounter();
}

void wdg_wide(void)
{
    iwdg_set_reload(WDG_RELOAD_WIDE);
}

void wdg_normal(void)
{
    iwdg_set_reload(WDG_RELOAD_NORMAL);
}

/* ==================== Flash 操作 ==================== */

static u32 flash_sector_of(u32 addr)
{
    u32 i;
    for (i = 0; i < 11; i++) {
        if (addr < s_sector_addr[i + 1]) return i;
    }
    return 11;
}

/* 按扇区擦除 [start, end) */
static void flash_erase_range(u32 start, u32 end)
{
    u32 s0 = flash_sector_of(start);
    u32 s1 = flash_sector_of(end - 1);
    u32 s;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    for (s = s0; s <= s1; s++) {
        FLASH_EraseSector(s, VoltageRange_3);
    }
    FLASH_Lock();
}

/* 写入并回读校验, 返回0成功 */
static int flash_program_verify(u32 addr, const u8 *data, u32 len)
{
    u32 i = 0, k;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    while (i < len) {
        u32 w = 0xFFFFFFFFUL;
        for (k = 0; k < 4 && i + k < len; k++) {
            w &= ~((u32)0xFF << (8 * k));
            w |= (u32)data[i + k] << (8 * k);
        }
        FLASH_ProgramWord(addr + i, w);
        if (*(volatile u32 *)(addr + i) != w) {
            FLASH_Lock();
            return -1;
        }
        i += 4;
    }
    FLASH_Lock();
    return 0;
}

/* ==================== 元数据 ==================== */

static void meta_write_ready(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, magic),       META_MAGIC);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, size),        s_size);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, crc32),       s_crc);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, app_version), s_ver);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, ready_flag),  META_FLAG_SET);
    FLASH_Lock();
}

void ota_init(void)
{
    const ota_meta_t *m = (const ota_meta_t *)META_ADDR;

    s_active = 0;
    s_commit_pending = 0;
    s_commit_cnt = 0;

    if (m->magic != META_MAGIC) {
        printf("[OTA] 元数据: 空 (未做过OTA)\r\n");
        return;
    }

    printf("[OTA] 元数据: size=%u crc=%08X ver=%u ready=%lu running=%lu commit=%lu\r\n",
           (unsigned)m->size, (unsigned)m->crc32, (unsigned)m->app_version,
           (unsigned long)m->ready_flag, (unsigned long)m->running_flag,
           (unsigned long)m->commit_flag);

    s_size = m->size;            /* 提交时要把版本号等信息写回去 */
    s_crc  = m->crc32;
    s_ver  = m->app_version;

    /* 由 Bootloader 装入槽位A后正在试用: 运行正常就要提交, 否则试满后回滚 */
    if (m->running_flag == META_FLAG_SET &&
        m->ready_flag   != META_FLAG_SET &&
        m->commit_flag  != META_FLAG_SET) {
        s_commit_pending = 1;
        printf("[OTA] 新固件试用中, 稳定运行5秒后提交\r\n");
    }
}

/* 由 wdg_task 每秒调用一次 (此时已确认各任务都活着) */
void ota_commit_tick(void)
{
    if (!s_commit_pending) return;
    if (++s_commit_cnt < 5) return;

    s_commit_pending = 0;
    printf("[OTA] 运行正常, 提交新固件(擦除元数据区)...\r\n");

    wdg_wide();                                  /* 擦除会阻塞数秒 */
    flash_erase_range(META_ADDR, META_ADDR + 4096);
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, magic),       META_MAGIC);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, app_version), s_ver);
    FLASH_ProgramWord(META_ADDR + offsetof(ota_meta_t, commit_flag), META_FLAG_SET);
    FLASH_Lock();
    wdg_normal();

    printf("[OTA] 提交完成, 以后不再回滚\r\n");
}

/* ==================== MQTT 消息提取 ==================== */

/* ESP-01S AT固件推送格式: +MQTTSUBRECV:<link>,"<topic>",<len>,<payload>\r\n
 * 从接收缓冲区取出第一条完整消息, 并把该行从缓冲区移走。
 * 返回1=取到一条消息, 0=暂无完整消息。 */

static void rx_shift(u16 n)
{
    u16 k, cnt;
    u32 primask = __get_PRIMASK();

    __disable_irq();                     /* 移位期间禁止接收中断写入 */
    cnt = ESP8266_RX_CNT;
    if (n > cnt) n = cnt;
    for (k = n; k < cnt; k++) ESP8266_RX_BUF[k - n] = ESP8266_RX_BUF[k];
    ESP8266_RX_CNT = cnt - n;
    __set_PRIMASK(primask);
}

static u8 ota_take_msg(char *topic, u16 tsz, char *pay, u16 psz)
{
    const char *mark = "+MQTTSUBRECV:";
    const u16   mlen = 13;
    u16 cnt = ESP8266_RX_CNT;
    u16 i, j, e, n, t0, t1, p0, p1;

    if (cnt < mlen) return 0;

    for (i = 0; i + mlen <= cnt; i++) {           /* 找标记 */
        for (j = 0; j < mlen; j++) {
            if (ESP8266_RX_BUF[i + j] != (u8)mark[j]) break;
        }
        if (j == mlen) break;
    }
    if (i + mlen > cnt) return 0;

    e = i + mlen;
    while (e < cnt && ESP8266_RX_BUF[e] != '\r') e++;   /* 行尾 */
    if (e >= cnt) return 0;                             /* 行未收全 */

    n = e + 1;
    if (n < cnt && ESP8266_RX_BUF[n] == '\n') n++;

    p0 = i + mlen;
    while (p0 < e && ESP8266_RX_BUF[p0] != '"') p0++;   /* topic 起始引号 */
    if (p0 < e) {
        t0 = ++p0;
        while (p0 < e && ESP8266_RX_BUF[p0] != '"') p0++;
        t1 = p0;
        while (p0 < e && ESP8266_RX_BUF[p0] != ',') p0++;  /* 长度字段后的逗号 */
        if (p0 < e && t1 > t0) {
            p1 = p0 + 1;                                    /* payload 起点 */
            j = 0;
            for (i = t0; i < t1 && j < tsz - 1; i++) topic[j++] = ESP8266_RX_BUF[i];
            topic[j] = 0;
            j = 0;
            for (i = p1; i < e && j < psz - 1; i++) pay[j++] = ESP8266_RX_BUF[i];
            pay[j] = 0;
            rx_shift(n);
            return 1;
        }
    }
    rx_shift(n);        /* 格式异常: 丢弃该行, 返回0让上层继续等 */
    return 0;
}

/* ==================== 应答 ==================== */

static void ota_pub_ack(const char *msg)
{
    char cmd[96];

    sprintf(cmd, "AT+MQTTPUB=0,\"" OTA_ACK_TOPIC "\",\"%s\",0,0", msg);
    esp8266_clear_rxbuf();
    esp8266_send_cmd(cmd);
    esp8266_wait_string("OK", 1000);
}

static void ota_ack_ok(u32 seq)
{
    char m[24];
    sprintf(m, "R,OK,%u", (unsigned)seq);
    ota_pub_ack(m);
}

/* 是否正在下载固件 (下载期间暂停MQTT数据上报, 避免和升级数据抢ESP模块) */
u8 ota_is_active(void)
{
    return s_active;
}

/* ==================== 协议解析 ==================== */

static const char *parse_dec(const char *p, u32 *out)
{
    u32 v = 0;
    u8  any = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (u32)(*p - '0'); p++; any = 1; }
    if (!any) return 0;
    *out = v;
    return p;
}

static const char *parse_hex8(const char *p, u32 *out)
{
    u32 v = 0;
    int i;
    for (i = 0; i < 8; i++) {
        char c = p[i];
        if      (c >= '0' && c <= '9') v = (v << 4) | (u32)(c - '0');
        else if (c >= 'A' && c <= 'F') v = (v << 4) | (u32)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') v = (v << 4) | (u32)(c - 'a' + 10);
        else return 0;
    }
    *out = v;
    return p + 8;
}

static int hex2bin(const char *h, u8 *out, u32 n)
{
    u32 i;
    for (i = 0; i < n; i++) {
        u8 v = 0;
        int k;
        for (k = 0; k < 2; k++) {
            char c = h[i * 2 + k];
            u8 d;
            if      (c >= '0' && c <= '9') d = (u8)(c - '0');
            else if (c >= 'A' && c <= 'F') d = (u8)(c - 'A' + 10);
            else if (c >= 'a' && c <= 'f') d = (u8)(c - 'a' + 10);
            else return -1;
            v = (u8)((v << 4) | d);
        }
        out[i] = v;
    }
    return 0;
}

/* 擦除暂存区: 槽位B 全部 + 元数据区所在扇区 (擦完元数据也是干净的) */
static int ota_erase_staging(void)
{
    printf("[OTA] 擦除暂存区(槽位B + 元数据扇区), 约需数秒...\r\n");
    wdg_wide();
    flash_erase_range(STAGING_ADDR, STAGING_ADDR + APP_MAX_SIZE);
    wdg_normal();
    printf("[OTA] 擦除完成\r\n");
    return 0;
}

static void ota_handle(const char *p)
{
    if (p[0] == 'B' && p[1] == ',') {
        /* ---- 开始 ---- */
        const char *q;
        u32 size, crc, ver;
        if (!(q = parse_dec(p + 2, &size)) || *q != ',') { ota_pub_ack("R,ERR,BF"); return; }
        if (!(q = parse_hex8(q + 1, &crc))  || *q != ',') { ota_pub_ack("R,ERR,BC"); return; }
        if (!parse_dec(q + 1, &ver))                      { ota_pub_ack("R,ERR,BV"); return; }
        if (size == 0 || size > APP_MAX_SIZE) {
            printf("[OTA] 固件大小非法: %u\r\n", (unsigned)size);
            ota_pub_ack("R,ERR,BS");
            return;
        }
        s_active = 0;
        s_size = size;
        s_crc  = crc;
        s_ver  = ver;
        s_seq  = 0;
        printf("[OTA] 开始接收: %u 字节, CRC=%08X, 版本=%u\r\n",
               (unsigned)size, (unsigned)crc, (unsigned)ver);
        ota_erase_staging();
        s_active = 1;
        ota_pub_ack("R,BEGIN");
    }
    else if (p[0] == 'D' && p[1] == ',') {
        /* ---- 数据块 ---- */
        const char *q, *h;
        u32 seq, off, n;
        if (!s_active)                       { ota_pub_ack("R,ERR,NA");  return; }
        if (!(q = parse_dec(p + 2, &seq)) || *q != ',') { ota_pub_ack("R,ERR,DF"); return; }
        h = q + 1;
        n = (u32)strlen(h) / 2;
        if (n == 0 || n > OTA_CHUNK_SIZE)    { ota_pub_ack("R,ERR,DL");  return; }
        if (seq < s_seq) { ota_ack_ok(seq);  return; }   /* 确认丢了导致的重发: 重新确认 */
        off = seq * OTA_CHUNK_SIZE;
        if (seq > s_seq || off + n > s_size) { ota_pub_ack("R,ERR,SEQ"); return; }
        if (hex2bin(h, s_chunk, n) != 0)     { ota_pub_ack("R,ERR,DH");  return; }
        if (flash_program_verify(STAGING_ADDR + off, s_chunk, n) != 0) {
            printf("[OTA] 写Flash失败: seq=%u\r\n", (unsigned)seq);
            ota_pub_ack("R,ERR,DP");
            return;
        }
        s_seq++;
        ota_ack_ok(seq);
    }
    else if (p[0] == 'E') {
        /* ---- 结束: 校验 ---- */
        u32 calc;
        char m[32];
        if (!s_active) { ota_pub_ack("R,ERR,NA"); return; }
        calc = crc32((const u8 *)STAGING_ADDR, s_size);
        printf("[OTA] 接收完成: 期望CRC=%08X 实际CRC=%08X\r\n",
               (unsigned)s_crc, (unsigned)calc);
        if (calc != s_crc) {
            s_active = 0;
            sprintf(m, "R,FAIL,%08X", (unsigned)calc);
            ota_pub_ack(m);
            printf("[OTA] CRC不匹配, 丢弃(请重发)\r\n");
            return;
        }
        meta_write_ready();          /* magic/size/crc/ver + ready_flag */
        s_active = 0;
        printf("[OTA] 校验通过, 元数据已写入 ready_flag\r\n");
        ota_pub_ack("R,DONE");
        delay_ms(300);
        printf("[OTA] 复位, 由 Bootloader 安装新固件...\r\n");
        delay_ms(300);
        NVIC_SystemReset();
    }
    else if (p[0] == 'A') {
        s_active = 0;
        printf("[OTA] 上位机放弃本次升级\r\n");
        ota_pub_ack("R,ABORT");
    }
}

/* ==================== 对外轮询接口 ==================== */

void ota_poll(void)
{
    static char topic[24];
    static char pay[400];
    u8 guard = 0;

    while (guard++ < 4 && ota_take_msg(topic, sizeof(topic), pay, sizeof(pay))) {
        if (strcmp(topic, OTA_TOPIC) == 0) {
            ota_handle(pay);
        } else {
            app_on_mqtt_msg(topic, pay);   /* 其它主题(阀门控制)交给 main.c */
        }
    }

    /* 兜底: 缓冲区快满却取不出完整消息(半截数据/垃圾数据), 清掉防止后续消息被挤丢 */
    if (ESP8266_RX_CNT >= ESP8266_RX_BUF_SIZE - 2) esp8266_clear_rxbuf();
}