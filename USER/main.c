#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "touch.h"
#include "can.h"
#include "esp8266.h"
#include "wifi_cfg.h"   // WiFi账号/密码/MQTT服务器IP (私有配置, 已gitignore不入库)
#include "stdio.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lvgl.h"
#include "lv_port.h"
#include "ui.h"
#include "ota.h"        // OTA升级: MQTT分块下载 + 元数据/看门狗
#include "string.h"

// F407 CAN -> MQTT 网关 (正点原子STM32F407探索者 + TJA1050 + ESP-01S)  +  FreeRTOS
//
// CAN1: PA11=RX, PA12=TX, 500kbps, FIFO0 中断接收
// F103 传感器节点发送温湿度/烟雾数据帧 (ID=0x0001, 标准帧) + 机器参数帧 (ID=0x0002)
// 本程序: 接收CAN帧 -> LCD显示 -> 通过ESP-01S发布到MQTT服务器
//
// LCD 为 ST7789V (240x320, 16bit并口):
//   - 驱动 lcd.c 的ID自动识别读不到它, 所以用 lcd_st7789_init() 强制初始化
//
// FreeRTOS 任务划分:
//   mqtt_task(pri=1): 上电连WiFi/MQTT + 轮询控制指令/OTA消息 + 发布数据 (不再阻塞界面)
//   app_task(pri=2):  触摸 + CAN接收 + LCD显示
//   led_task(pri=1):  LED心跳
//   wdg_task(pri=3):  看门狗: 每秒检查上面3个任务都活着才喂狗(否则让IWDG复位)

// ===== MQTT 配置 (WiFi账号/密码/MQTT服务器IP 见 wifi_cfg.h) =====
#define MQTT_CLIENT_ID  "F407_Gateway"     // MQTT客户端ID,可任意取
#define MQTT_TOPIC      "sensor/data"      // 发布主题
#define MQTT_CTRL_TOPIC "sensor/ctrl"      // 订阅主题 (QT上位机下发阀门控制: "1"=开阀 "0"=关阀)

// ===== 运行状态 =====
u8 mqtt_connected = 0;   // 0=MQTT未连接(不阻塞,照常处理CAN), 1=已连接可发布

// ===== 任务间共享数据 =====
volatile sensor_frame_t  g_sensor;          // 最近一帧传感器数据 (app_task写, mqtt_task读)
volatile machine_frame_t g_machine;         // 最近一帧机器参数
volatile u8 g_have_sensor = 0;              // 是否已收到过传感器帧
volatile u8 g_machine_updated = 0;          // 新机器帧到达标志 (mqtt_task发布后清零)

// ===== 任务活性心跳 (各任务循环末尾更新, wdg_task 检查) =====
volatile TickType_t g_hb_app  = 0;
volatile TickType_t g_hb_mqtt = 0;
volatile TickType_t g_hb_led  = 0;
// mqtt_connect 上电要跑约20秒(AT指令等待), 期间不要求 mqtt 心跳 (失败也有超时上限)
volatile u8 g_mqtt_busy = 1;

// ST7789V 初始化 (必须在 LCD_Init() 之后调用)
// 注意: lcddev.id 设为 0x9341 是关键! ST7789与ILI9341的窗口/光标/GRAM命令完全兼容,
//       设成9341后所有 LCD_* 绘制函数才会走正确的命令分支(否则走ILI9320老式分支全部失效)
void lcd_st7789_init(void)
{
    // 覆盖 LCD 参数 (id故意用9341, 让驱动按ILI9341方式操作, 兼容ST7789)
    lcddev.id = 0x9341;
    lcddev.width = 240;
    lcddev.height = 320;
    lcddev.dir = 0;
    lcddev.wramcmd = 0x2C;   // 写GRAM命令
    lcddev.setxcmd = 0x2A;   // 设置列地址
    lcddev.setycmd = 0x2B;   // 设置行地址

    LCD->LCD_REG = 0x01; delay_ms(120);   // 软件复位
    LCD->LCD_REG = 0x11; delay_ms(120);   // 退出睡眠
    LCD->LCD_REG = 0x36; LCD->LCD_RAM = 0x00;  // MADCTL 竖屏
    LCD->LCD_REG = 0x3A; LCD->LCD_RAM = 0x55;  // 16bit/像素
    LCD->LCD_REG = 0xB2; LCD->LCD_RAM = 0x0C; LCD->LCD_RAM = 0x0C; LCD->LCD_RAM = 0x00; LCD->LCD_RAM = 0x33; LCD->LCD_RAM = 0x33; // 边框
    LCD->LCD_REG = 0xB7; LCD->LCD_RAM = 0x35;  // 门控
    LCD->LCD_REG = 0xBB; LCD->LCD_RAM = 0x19;  // VCOM
    LCD->LCD_REG = 0xC0; LCD->LCD_RAM = 0x2C;  // LCM控制
    LCD->LCD_REG = 0xC2; LCD->LCD_RAM = 0x01;  // VDV/VRH命令使能
    LCD->LCD_REG = 0xC3; LCD->LCD_RAM = 0x12;  // VRH
    LCD->LCD_REG = 0xC4; LCD->LCD_RAM = 0x20;  // VDV
    LCD->LCD_REG = 0xC6; LCD->LCD_RAM = 0x0F;  // 帧率
    LCD->LCD_REG = 0xD0; LCD->LCD_RAM = 0xA4; LCD->LCD_RAM = 0xA1;  // 电源控制1
    // 正伽马
    LCD->LCD_REG = 0xE0; LCD->LCD_RAM = 0xD0; LCD->LCD_RAM = 0x04; LCD->LCD_RAM = 0x0D; LCD->LCD_RAM = 0x11; LCD->LCD_RAM = 0x13;
    LCD->LCD_RAM = 0x2B; LCD->LCD_RAM = 0x3F; LCD->LCD_RAM = 0x54; LCD->LCD_RAM = 0x4C; LCD->LCD_RAM = 0x18; LCD->LCD_RAM = 0x0D;
    LCD->LCD_RAM = 0x0B; LCD->LCD_RAM = 0x1F; LCD->LCD_RAM = 0x23;
    // 负伽马
    LCD->LCD_REG = 0xE1; LCD->LCD_RAM = 0xD0; LCD->LCD_RAM = 0x04; LCD->LCD_RAM = 0x0C; LCD->LCD_RAM = 0x11; LCD->LCD_RAM = 0x13;
    LCD->LCD_RAM = 0x2C; LCD->LCD_RAM = 0x3F; LCD->LCD_RAM = 0x44; LCD->LCD_RAM = 0x51; LCD->LCD_RAM = 0x2F; LCD->LCD_RAM = 0x1F;
    LCD->LCD_RAM = 0x1F; LCD->LCD_RAM = 0x20; LCD->LCD_RAM = 0x23;
    LCD->LCD_REG = 0x20;                   // 反色显示关 (此屏不需要反色)
    LCD->LCD_REG = 0x29;                   // 开显示
    delay_ms(20);
}

// WiFi连接 + MQTT连接 (AT指令流程, 失败不阻塞主流程)
// 注意: 该函数在 mqtt_task 中运行, 内部 delay_ms 会 vTaskDelay 让出 CPU
void mqtt_connect(void)
{
    u8  got = 0;
    u16 t;
    // 各步骤结果 (用于最后一行汇总, 避免串口被刷屏时看不到中间日志)
    u8  r_reset = 0, r_at = 0, r_sta = 0, r_wifi = 0, r_cfg = 0, r_conn = 0, r_sub = 0, r_ota = 0;

    printf("\r\n========== ESP-01S WiFi/MQTT 连接 ==========\r\n");
    delay_ms(1500);           // 等待ESP-01S上电启动

    // 0. 复位模块 (解决模块卡死/状态异常)
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT+RST");
    r_reset = esp8266_wait_string("ready", 3000);
    if (r_reset)
        printf("[0] 模块复位 OK\r\n");
    else
        printf("[0] 复位无响应(继续尝试)\r\n");
    delay_ms(500);

    // 1. 验证模块
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT");
    r_at = esp8266_wait_string("OK", 1000);
    if (r_at)
        printf("[1] AT OK\r\n");
    else
    {
        // 打印RX缓冲: 没数据=供电/接线问题, 乱码=波特率问题
        printf("[1] AT 无响应! RX_CNT=%d\r\n", (int)ESP8266_RX_CNT);
        esp8266_print_rxbuf();
    }

    // 2. STA模式
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT+CWMODE=1");
    r_sta = esp8266_wait_string("OK", 1000);
    if (r_sta)
        printf("[2] STA模式 OK\r\n");
    else
    {
        printf("[2] CWMODE 失败! RX_CNT=%d\r\n", (int)ESP8266_RX_CNT);
        esp8266_print_rxbuf();
    }

    // 3. 连接路由器 (最多等8秒)
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PWD "\"");
    got = 0;
    for (t = 0; t < 80; t++)
    {
        // 新固件连接成功返回 WIFI GOT IP, 旧固件返回 OK
        if (esp8266_wait_string("GOT IP", 100) || esp8266_wait_string("OK", 100))
        {
            got = 1;
            break;
        }
    }
    r_wifi = got;
    if (r_wifi) printf("[3] WiFi 连接成功\r\n");
    else        printf("[3] WiFi 连接失败/超时\r\n");

    // 4. MQTT 客户端配置
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT+MQTTUSERCFG=0,1,\"" MQTT_CLIENT_ID "\",\"\",\"\",0,0,\"\"");
    r_cfg = esp8266_wait_string("OK", 1000);
    if (r_cfg)
        printf("[4] MQTT用户配置 OK\r\n");
    else
    {
        printf("[4] MQTTUSERCFG 失败! RX_CNT=%d\r\n", (int)ESP8266_RX_CNT);
        esp8266_print_rxbuf();
    }

    // 5. 连接 MQTT 服务器 (最多等5秒)
    esp8266_clear_rxbuf();
    esp8266_send_cmd("AT+MQTTCONN=0,\"" MQTT_BROKER_IP "\",1883,0");
    got = 0;
    for (t = 0; t < 50; t++)
    {
        // 连接成功返回 CONNECT, 失败返回 ERROR
        if (esp8266_wait_string("CONNECT", 100) || esp8266_wait_string("OK", 100))
        {
            got = 1;
            break;
        }
    }
    r_conn = got;
    if (r_conn)
    {
        printf("[5] MQTT 服务器连接成功\r\n");
        mqtt_connected = 1;   // 置连接标志, 主循环开始发布数据
    }
    else
    {
        printf("[5] MQTT 连接失败/超时\r\n");
        esp8266_print_rxbuf();
    }

    // 6. 订阅 OTA 升级主题 (接收QT上位机下发的固件升级指令)
    // 注意: ESP-01S AT固件对 "#" 通配符订阅返回OK但不生效, 且多主题订阅不可靠;
    //       当前只订阅 ota/fw 单主题, 优先打通OTA (QT阀门按钮暂不可用, 屏幕阀门按钮不受影响)
    if (mqtt_connected)
    {
        esp8266_clear_rxbuf();
        esp8266_send_cmd("AT+MQTTSUB=0,\"" OTA_TOPIC "\",0");
        r_sub = esp8266_wait_string("OK", 1000);
        if (r_sub)
            printf("[6] OTA 主题订阅成功\r\n");
        else
        {
            printf("[6] OTA 订阅失败! RX_CNT=%d\r\n", (int)ESP8266_RX_CNT);
            esp8266_print_rxbuf();
        }
        r_ota = r_sub;
    }

    // 7. 诊断: 打印AT固件版本 + 自发自收测试 (定位订阅无效问题, 之后删除)
    if (mqtt_connected && r_ota)
    {
        printf("[7] 查询AT固件版本:\r\n");
        esp8266_clear_rxbuf();
        esp8266_send_cmd("AT+GMR");
        delay_ms(800);
        esp8266_print_rxbuf();

        printf("[7] 自发自收测试: 发布一条到 ota/fw, 等它回推...\r\n");
        esp8266_clear_rxbuf();
        esp8266_send_cmd("AT+MQTTPUB=0,\"" OTA_TOPIC "\",\"SELFTEST\",0,0");
        esp8266_wait_string("OK", 2000);
        delay_ms(1500);              // 等 +MQTTSUBRECV 回推到达
        if (esp8266_has_pending_msg())
        {
            printf("[7] 自发自收: 收到回推! 订阅功能正常\r\n");
        }
        else
        {
            printf("[7] 自发自收: 未收到回推! ESP-01S MQTT订阅功能异常\r\n");
            esp8266_print_rxbuf();
        }
        esp8266_clear_rxbuf();       // 清理测试残留, 进入主循环
    }

    // ===== 汇总行: 一条看完全部步骤 (串口刷屏也能看清) =====
    printf("\r\n===== ESP-01S 连接汇总 =====\r\n");
    printf("复位:%s  AT:%s  STA:%s  WiFi:%s  MQTT配置:%s  MQTT连接:%s  OTA订阅:%s\r\n",
           r_reset ? "OK" : "--",  r_at ? "OK" : "--",  r_sta ? "OK" : "--",
           r_wifi ? "OK" : "FAIL", r_cfg ? "OK" : "--", r_conn ? "OK" : "FAIL",
           r_ota ? "OK" : "--");
    printf("当前连接状态: %s\r\n", mqtt_connected ? "已连接(可发布)" : "未连接");

    g_mqtt_busy = 0;   // 连接流程结束, 之后由 wdg_task 监督 mqtt 任务心跳
}

// ===== 屏幕按钮已由 LVGL 接管 (见 ui.c); 原裸机 GUI_DrawButtons/GUI_TapButton/GUI_FeedbackPressed 已删除 =====

// 发布一帧传感器+机器数据到 MQTT
// 实际发送的AT指令形如:
//   AT+MQTTPUB=0,"sensor/data","{\"temp\":25.34\,\"humi\":56.78\,\"smoke\":90.00\,\"rpm\":2000\,\"press\":0.75\,\"vib\":2.50\,\"state\":1\,\"valve\":0}",0,0
void mqtt_publish_sensor(sensor_frame_t *frame, machine_frame_t *machine)
{
    char cmd[256];

    if (!mqtt_connected) return;   // MQTT未连接: 不阻塞, 只处理CAN/LCD

    sprintf(cmd,
            "AT+MQTTPUB=0,\"" MQTT_TOPIC "\",\"{\\\"temp\\\":%d.%02d\\,\\\"humi\\\":%d.%02d\\,\\\"smoke\\\":%d.%02d\\,\\\"rpm\\\":%d\\,\\\"press\\\":%d.%02d\\,\\\"vib\\\":%d.%02d\\,\\\"state\\\":%d\\,\\\"valve\\\":%d}\",0,0",
            frame->temp_int, frame->temp_dec,
            frame->humi_int, frame->humi_dec,
            frame->smoke_int, frame->smoke_dec,
            (int)machine->spindle_rpm,
            machine->pressure_int, machine->pressure_dec,
            machine->vib_int, machine->vib_dec,
            machine->state,
            (frame->status >> 2) & 1);   // 阀门状态: 传感器帧status bit2

    esp8266_clear_rxbuf();
    esp8266_send_cmd(cmd);
    if (esp8266_wait_string("OK", 1000))
        printf("[MQTT] 发布成功\r\n");
    else
    {
        printf("[MQTT] 发布失败, 断开MQTT\r\n");
        mqtt_connected = 0;   // 连接已断, 停止发布尝试, 避免主循环每帧阻塞
    }
}

// 处理非OTA主题的MQTT消息 (由 ota_poll() 从ESP接收缓冲解析出 topic/payload 后回调)
// 当前只有阀门控制主题: QT上位机发布 "1"(开阀) / "0"(关阀), F407 收到后转发给F103
void app_on_mqtt_msg(const char *topic, const char *payload)
{
    u16  n;
    char val;

    if (strcmp(topic, MQTT_CTRL_TOPIC) != 0) return;

    n = (u16)strlen(payload);
    if (n == 0) return;
    val = payload[n - 1];      // 载荷最后一个字符即指令

    if (val == '1')
    {
        printf("[MQTT] 收到开阀指令 -> F103\r\n");
        if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_OPEN))
            printf("[MQTT] 开阀CAN发送失败!\r\n");
    }
    else if (val == '0')
    {
        printf("[MQTT] 收到关阀指令 -> F103\r\n");
        if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_CLOSE))
            printf("[MQTT] 关阀CAN发送失败!\r\n");
    }
}

// ===== FreeRTOS 钩子函数 =====
// 任务栈溢出 / 内存分配失败时进入死循环 (便于调试时定位)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    taskDISABLE_INTERRUPTS();
    GPIO_ResetBits(GPIOF, GPIO_Pin_10);   // LED1 常亮 = 栈溢出
    LCD_Clear(BLUE);                      // 蓝屏 = 任务栈溢出 (哪个任务见串口)
    printf("[RTOS] Stack Overflow: %s\r\n", pcTaskName);
    while (1);
}
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    GPIO_ResetBits(GPIOF, GPIO_Pin_10);   // LED1 常亮 = 内存分配失败
    LCD_Clear(WHITE);                     // 白屏 = RTOS堆内存分配失败
    printf("[RTOS] Malloc Failed!\r\n");
    while (1);
}

// ===== 诊断: 打印ESP接收缓冲的非应答内容 (纯"OK\r\n"应答不打印避免刷屏) =====
// 定位OTA收不到消息的问题后删除
static void rxdiag_print(void)
{
    static volatile u32 s_last = 0;
    u16 i, cnt = ESP8266_RX_CNT;
    u32 now = xTaskGetTickCount();

    if (cnt == 0) return;
    /* 纯 OK\r\n 应答垃圾: 不打印 */
    for (i = 0; i < cnt; i++) {
        u8 c = ESP8266_RX_BUF[i];
        if (c != 'O' && c != 'K' && c != '\r' && c != '\n' && c != 0) break;
    }
    if (i == cnt) return;
    /* MQTT推送必须立刻打印, 不受500ms限频约束(否则会被发布回显挤掉, 消息"隐形") */
    {
        u8 has_push = esp8266_has_pending_msg();
        if (!has_push && (u32)(now - s_last) < 500) return;
        s_last = now;
    }

    printf("[RX] cnt=%u: ", (unsigned)cnt);
    for (i = 0; i < cnt; i++) {
        u8 c = ESP8266_RX_BUF[i];
        printf("%c", (c >= 32 && c < 127) ? c : '.');
    }
    printf("\r\n");
}

// ===== 任务1: MQTT (优先级1) =====
// 上电连接 WiFi/MQTT (原先阻塞十几秒的代码挪到这里, 界面不再卡死);
// 之后统一解析ESP收到的MQTT消息(OTA升级 / 阀门控制) + 有新机器帧时发布数据
void mqtt_task(void *p)
{
    g_hb_mqtt = xTaskGetTickCount();
    mqtt_connect();                       // 内部耗时约20秒(g_mqtt_busy=1 期间免检心跳)
    g_hb_mqtt = xTaskGetTickCount();

    while (1)
    {
        static u32 s_last_resub = 0;

        // 周期重新订阅 OTA 主题(每5秒): 断线重连/会话替换后订阅可能丢失, 重订一次兜底
        // OTA下载中跳过, 避免AT应答和固件数据块抢ESP串口
        if (mqtt_connected && !ota_is_active() &&
            (u32)(xTaskGetTickCount() - s_last_resub) >= 5000)
        {
            s_last_resub = xTaskGetTickCount();
            esp8266_send_cmd("AT+MQTTSUB=0,\"" OTA_TOPIC "\",0");
            printf("[MQTT] 周期重订阅: %s\r\n",
                   esp8266_wait_string("OK", 1000) ? "OK" : "失败!");
        }

        rxdiag_print();   // 诊断打印 (ESP收到的原始数据, 定位OTA问题)

        // 取出ESP缓冲里所有完整的MQTT消息: OTA主题自己处理, 其它主题回调 main.c
        ota_poll();

        // 有新机器帧则发布一次合并数据 (app_task 收到帧后置标志)
        // OTA下载期间暂停上报: 别和升级数据抢ESP模块
        // 缓冲里有未处理的MQTT推送(可能是OTA帧)时也暂停发布, 否则 publish 的
        // clear_rxbuf 会把刚到达/半截的OTA帧清掉, 导致升级永远无法开始
        if (g_have_sensor && g_machine_updated && !ota_is_active() &&
            !esp8266_has_pending_msg())
        {
            mqtt_publish_sensor((sensor_frame_t*)&g_sensor, (machine_frame_t*)&g_machine);
            g_machine_updated = 0;
        }

        g_hb_mqtt = xTaskGetTickCount();  // 任务活性心跳
        vTaskDelay(50);
    }
}

// LVGL 阀门按钮回调: 界面按钮按下 -> 发送 CAN 控制帧到 F103
void ui_valve_can_cb(int32_t valve)
{
    if (valve == CAN_CTRL_VALVE_OPEN)
    {
        printf("[UI] 开阀命令 -> F103\r\n");
        if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_OPEN))
            printf("[UI] 开阀CAN发送失败!\r\n");
    }
    else
    {
        printf("[UI] 关阀命令 -> F103\r\n");
        if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_CLOSE))
            printf("[UI] 关阀CAN发送失败!\r\n");
    }
}

// ===== 任务2: 界面 (优先级2) =====
// CAN接收 + LVGL界面刷新 (触控由 LVGL indev 读取, 按钮由 ui.c 回调)
// LVGL 节拍与刷新: 每10ms 调 lv_timer_handler()
void app_task(void *p)
{
    sensor_frame_t frame;
    machine_frame_t machine;
    static sensor_frame_t last_sensor;   // 缓存最近一帧温湿度, 与机器帧合并上报
    static u8 have_sensor = 0;

    while (1)
    {
        // ---- CAN 接收: 传感器帧到了就刷传感器UI (不再依赖机器帧) ----
        if (CAN1_Receive_Frame(&frame))
        {
            last_sensor = frame;
            have_sensor = 1;

            // 注意: 此处不再 printf! app_task 与 mqtt_task 并发调用 printf
            // 会破坏 MicroLIB printf 内部状态导致 HardFault (已实测崩溃),
            // 传感器数据直接上 LCD, 高频串口打印只保留在 mqtt_task 一处。

            // ---- 更新 LVGL 传感器数据 (温湿度/烟雾) ----
            ui_update_sensor((float)frame.temp_int + (float)frame.temp_dec * 0.01f,
                             (float)frame.humi_int + (float)frame.humi_dec * 0.01f,
                             (float)frame.smoke_int + (float)frame.smoke_dec * 0.01f);
        }

        // ---- 机器参数帧到了就刷机器UI (两类帧解耦, 缺一不影响另一个) ----
        if (CAN1_Receive_MachineFrame(&machine))
        {
            // (同理不再 printf, 机器数据直接上 LCD)

            // ---- 更新 LVGL 机器数据 (振动按一位小数解析: vib_dec / 10) ----
            ui_update_machine((int32_t)machine.spindle_rpm,
                              (float)machine.pressure_int + (float)machine.pressure_dec * 0.01f,
                              (float)machine.vib_int + (float)machine.vib_dec * 0.1f);
            ui_update_status((int32_t)machine.state,
                             (have_sensor ? (last_sensor.status >> 2) & 1 : 0));

            // ---- 更新共享数据, 供 mqtt_task 发布 ----
            if (have_sensor)
            {
                g_sensor = last_sensor;
                g_machine = machine;
                g_have_sensor = 1;
                g_machine_updated = 1;
            }
        }

        // ---- MQTT 状态指示单独刷新 (跟随连接状态, 不依赖CAN帧) ----
        ui_update_mqtt(mqtt_connected ? true : false);

        // ---- LVGL 节拍 + 刷新 ----
        lv_timer_handler();

        g_hb_app = xTaskGetTickCount();   // 任务活性心跳
        vTaskDelay(10);
    }
}

// ===== 任务3: LED心跳 (优先级1) =====
void led_task(void *p)
{
    while (1)
    {
        GPIO_ResetBits(GPIOF, GPIO_Pin_9);    // LED0亮
        vTaskDelay(200);
        GPIO_SetBits(GPIOF, GPIO_Pin_9);      // LED0灭
        vTaskDelay(200);
        g_hb_led = xTaskGetTickCount();       // 任务活性心跳
    }
}

// ===== 任务4: 看门狗 (优先级3, 最高) =====
// 任务活性监督: 每秒检查 app/mqtt/led 三个任务的心跳都在3秒内更新才喂狗;
// 任何一个卡死就不再喂狗, 让 IWDG(约4秒) 复位系统。
// 另外: 系统连续健康运行5秒后, 提交试用中的新固件(OTA)。
void wdg_task(void *p)
{
    TickType_t now;

    wdg_init();      // 使能IWDG(约4秒), 并把启动阶段放宽的超时恢复正常
    printf("[WDG] 看门狗已使能(约4秒), 开始监督任务活性\r\n");

    while (1)
    {
        vTaskDelay(1000);
        now = xTaskGetTickCount();

        /* OTA下载期间 mqtt_task 会长时间阻塞(擦除/ACK/MQTT重连), 心跳必然超时,
           此时 IWDG 已由 ota 侧放宽到32.8秒兜底, 这里不再判 mqtt 卡死 */
        if (((g_mqtt_busy || ota_is_active()) || (now - g_hb_mqtt) <= 3000) &&
            (now - g_hb_app) <= 3000 &&
            (now - g_hb_led) <= 3000)
        {
            wdg_feed();
            ota_commit_tick();   // 试用固件: 健康运行足够久则提交
        }
        else
        {
            printf("[WDG] 任务卡死! app=%u mqtt=%u led=%u\r\n",
                   (unsigned)(now - g_hb_app),
                   (unsigned)(now - g_hb_mqtt),
                   (unsigned)(now - g_hb_led));
        }
    }
}

int main(void)
{
    /* Bootloader 跳转前执行了 __disable_irq() 且不会恢复, 这里必须重新开放中断,
       否则 FreeRTOS 调度器(SysTick/SVC/PendSV)无法工作。 */
    __enable_irq();

    delay_init(168);          // 延时函数初始化 (最先做, 后面都要用)
    uart_init(115200);        // 串口1: 尽早初始化, 便于定位卡在哪一步
    printf("\r\n[APP] main 入口\r\n");

    // Bootloader 启动 IWDG 时超时只有3秒, 而 App 初始化(LCD/LVGL/界面)可能超过3秒,
    // 所以先把 IWDG 超时临时放宽到约32秒; 调度器启动后由 wdg_task 恢复正常值。
    // (没有 Bootloader 时 IWDG 未使能, 这些写操作无影响)
    wdg_wide();
    printf("[APP] 1 看门狗已放宽\r\n");

    LED_Init();               // LED0=PF9, LED1=PF10 (低电平点亮)
    printf("[APP] 2 LED完成\r\n");

    LCD_Init();               // 初始化FSMC/GPIO (ID识别会失败, 正常)
    lcd_st7789_init();        // ST7789V 强制初始化
    printf("[APP] 3 LCD完成\r\n");

    TP_Init();                // 触摸屏 XPT2046 (软件SPI: 须在LCD之后, 避免FSMC重配PF0)
    printf("[APP] 4 触摸完成\r\n");

#if TP_DO_CALIB
    TP_Calibrate();           // 上电4点校准 (系数打印到串口; 固化后把 TP_DO_CALIB 改回0)
#endif

    CAN1_Init();              // CAN1 500kbps (PA11=RX, PA12=TX), FIFO0 中断接收
    printf("[APP] 5 CAN完成\r\n");

    esp8266_init(115200);     // ESP-01S 通过 USART3(PB10/PB11) 通信
    printf("[APP] 6 ESP8266完成\r\n");

    // ---- LVGL 界面初始化 ----
    lv_init();                // LVGL 核心初始化
    lv_port_init();           // 显示/触摸/节拍 端口
    printf("[APP] 7 LVGL完成\r\n");

    ui_init();                // 创建 HMI 界面控件
    ui_set_valve_cb(ui_valve_can_cb);  // 按钮 -> CAN 控制帧
    printf("[APP] 8 界面完成\r\n");

    printf("\r\n========== F407 CAN -> MQTT Gateway (FreeRTOS + LVGL) ==========\r\n");

    ota_init();               // 读OTA元数据: 判断是否在"试用新固件"(稳定后提交)

    // ---- 创建任务并启动调度器 ----
    {
        BaseType_t ok1 = xTaskCreate(mqtt_task, "mqtt", 1024, NULL, 1, NULL);  // 栈:1024字=4KB
        BaseType_t ok2 = xTaskCreate(app_task,  "app",   3072, NULL, 2, NULL); // 栈:3072字=12KB, LVGL全屏渲染+浮点格式化调用深, 6KB会溢出踩坏LVGL堆
        BaseType_t ok3 = xTaskCreate(led_task,  "led",   128, NULL, 1, NULL);
        BaseType_t ok4 = xTaskCreate(wdg_task,  "wdg",   512, NULL, 3, NULL);  // 栈:512字=2KB (含printf)
        if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
        {
            printf("[RTOS] 任务创建失败! ok1=%d ok2=%d ok3=%d ok4=%d\r\n",
                   (int)ok1, (int)ok2, (int)ok3, (int)ok4);
            GPIO_ResetBits(GPIOF, GPIO_Pin_10);          // LED1 常亮 = 致命错误
            LCD_Clear(RED);
            while (1);
        }
    }
    vTaskStartScheduler();    // 启动调度器, 永不返回

    // 走到这里 = 调度器启动失败 (致命)
    GPIO_ResetBits(GPIOF, GPIO_Pin_10);                // LED1 常亮
    LCD_Clear(RED);
    while (1);
}
