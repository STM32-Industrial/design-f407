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
//   mqtt_task(pri=1): 上电连WiFi/MQTT + 轮询控制指令 + 发布数据 (不再阻塞界面)
//   app_task(pri=2):  触摸 + CAN接收 + LCD显示
//   led_task(pri=1):  LED心跳

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
    u8  r_reset = 0, r_at = 0, r_sta = 0, r_wifi = 0, r_cfg = 0, r_conn = 0, r_sub = 0;

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

    // 6. 订阅控制主题 (接收QT上位机下发的阀门控制指令)
    if (mqtt_connected)
    {
        esp8266_clear_rxbuf();
        esp8266_send_cmd("AT+MQTTSUB=0,\"" MQTT_CTRL_TOPIC "\",0");
        r_sub = esp8266_wait_string("OK", 1000);
        if (r_sub)
            printf("[6] MQTT 订阅成功\r\n");
        else
        {
            printf("[6] MQTT 订阅失败! RX_CNT=%d\r\n", (int)ESP8266_RX_CNT);
            esp8266_print_rxbuf();
        }
    }

    // ===== 汇总行: 一条看完全部步骤 (串口刷屏也能看清) =====
    printf("\r\n===== ESP-01S 连接汇总 =====\r\n");
    printf("复位:%s  AT:%s  STA:%s  WiFi:%s  MQTT配置:%s  MQTT连接:%s  MQTT订阅:%s\r\n",
           r_reset ? "OK" : "--",  r_at ? "OK" : "--",  r_sta ? "OK" : "--",
           r_wifi ? "OK" : "FAIL", r_cfg ? "OK" : "--", r_conn ? "OK" : "FAIL",
           r_sub ? "OK" : "--");
    printf("当前连接状态: %s\r\n", mqtt_connected ? "已连接(可发布)" : "未连接");
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

// 轮询MQTT下发的阀门控制指令
// QT上位机向 MQTT_CTRL_TOPIC 发布 "1"(开阀) 或 "0"(关阀), F407 收到后转发给F103
// 解析 AT 固件的异步推送: +MQTTSUBRECV:0,"sensor/ctrl",1,<data>\r\n
// 返回 CAN_CTRL_VALVE_OPEN/CLOSE, 无新指令返回 0xFF
u8 mqtt_poll_ctrl(void)
{
    static const char mark[] = "+MQTTSUBRECV:";
    u16 i, j, m;
    u8  val;

    if (ESP8266_RX_CNT < sizeof(mark) - 1) return 0xFF;

    for (i = 0; i + sizeof(mark) - 1 <= ESP8266_RX_CNT; i++)
    {
        for (j = 0; mark[j]; j++)
            if (ESP8266_RX_BUF[i + j] != mark[j]) break;
        if (mark[j] != 0) continue;            // 本位置不匹配, 继续找

        // 定位本行结尾 \r
        m = i;
        while (m < ESP8266_RX_CNT && ESP8266_RX_BUF[m] != '\r') m++;
        if (m >= ESP8266_RX_CNT) return 0xFF;  // 行未收全, 等下一轮

        if (m == i) { esp8266_clear_rxbuf(); return 0xFF; }
        val = ESP8266_RX_BUF[m - 1];           // 行尾前一个字符即数据
        esp8266_clear_rxbuf();                 // 处理完清除, 避免重复触发
        if (val == '1') return CAN_CTRL_VALVE_OPEN;
        if (val == '0') return CAN_CTRL_VALVE_CLOSE;
        return 0xFF;
    }
    return 0xFF;
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

// ===== 任务1: MQTT (优先级1) =====
// 上电连接 WiFi/MQTT (原先阻塞十几秒的代码挪到这里, 界面不再卡死);
// 之后轮询控制指令 + 有新机器帧时发布数据
void mqtt_task(void *p)
{
    u8 ctrl_cmd;

    mqtt_connect();

    while (1)
    {
        // MQTT下发的阀门控制指令 (QT上位机 -> F103)
        ctrl_cmd = mqtt_poll_ctrl();
        if (ctrl_cmd == CAN_CTRL_VALVE_OPEN)
        {
            printf("[MQTT] 收到开阀指令 -> F103\r\n");
            if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_OPEN))
                printf("[MQTT] 开阀CAN发送失败!\r\n");
        }
        else if (ctrl_cmd == CAN_CTRL_VALVE_CLOSE)
        {
            printf("[MQTT] 收到关阀指令 -> F103\r\n");
            if (!CAN1_Send_CtrlValve(CAN_CTRL_VALVE_CLOSE))
                printf("[MQTT] 关阀CAN发送失败!\r\n");
        }

        // 有新机器帧则发布一次合并数据 (app_task 收到帧后置标志)
        if (g_have_sensor && g_machine_updated)
        {
            mqtt_publish_sensor((sensor_frame_t*)&g_sensor, (machine_frame_t*)&g_machine);
            g_machine_updated = 0;
        }

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
    }
}

int main(void)
{
    delay_init(168);          // 延时函数初始化
    LED_Init();               // LED0=PF9, LED1=PF10 (低电平点亮)
    uart_init(115200);        // 串口1: 调试打印
    LCD_Init();               // 初始化FSMC/GPIO (ID识别会失败, 正常)
    lcd_st7789_init();        // ST7789V 强制初始化

    TP_Init();                // 触摸屏 XPT2046 (软件SPI: 须在LCD之后, 避免FSMC重配PF0)

#if TP_DO_CALIB
    TP_Calibrate();           // 上电4点校准 (系数打印到串口; 固化后把 TP_DO_CALIB 改回0)
#endif

    CAN1_Init();              // CAN1 500kbps (PA11=RX, PA12=TX), FIFO0 中断接收

    esp8266_init(115200);     // ESP-01S 通过 USART3(PB10/PB11) 通信

    // ---- LVGL 界面初始化 ----
    lv_init();                // LVGL 核心初始化
    lv_port_init();           // 显示/触摸/节拍 端口

    ui_init();                // 创建 HMI 界面控件
    ui_set_valve_cb(ui_valve_can_cb);  // 按钮 -> CAN 控制帧

    printf("\r\n========== F407 CAN -> MQTT Gateway (FreeRTOS + LVGL) ==========\r\n");

    // ---- 创建任务并启动调度器 ----
    {
        BaseType_t ok1 = xTaskCreate(mqtt_task, "mqtt", 1024, NULL, 1, NULL);  // 栈:1024字=4KB
        BaseType_t ok2 = xTaskCreate(app_task,  "app",   3072, NULL, 2, NULL); // 栈:3072字=12KB, LVGL全屏渲染+浮点格式化调用深, 6KB会溢出踩坏LVGL堆
        BaseType_t ok3 = xTaskCreate(led_task,  "led",   128, NULL, 1, NULL);
        if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS)
        {
            printf("[RTOS] 任务创建失败! ok1=%d ok2=%d ok3=%d\r\n", (int)ok1, (int)ok2, (int)ok3);
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
