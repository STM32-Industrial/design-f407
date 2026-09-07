/**
 * @file ui.h
 * @brief CAN->MQTT 机器监控网关 - 工业 HMI 仪表盘（纯 LVGL 9.5 API，可原样移植到 STM32F407）
 *
 * 设计规范：
 *   - 页面 240x320（对齐 ST7789V 竖屏）
 *   - 深色主题：#1E2228 背景 / #262B34 卡片 / #343B47 边框
 *   - 强调色（青色系）：#2EC4B6
 *   - 状态色：绿 #2ECC71 / 橙 #F5A623 / 红 #E74C3C
 *   - 仅使用 LVGL 内置 montserrat 字体（12/14/16/20）
 *   - 全部使用绝对定位（lv_obj_set_pos / lv_obj_align），不使用 flex/grid
 *   - 不使用阴影、透明度等耗性能效果，保证 F407 上流畅
 *
 * 移植说明：
 *   - 本文件与 ui.h 不依赖任何平台函数，仅依赖 lvgl.h
 *   - 数据更新函数可被外部任务以 <=10ms 周期调用，内部无阻塞/延时
 *   - ui_update_* 只更新对应控件句柄，不重建整个 UI
 */

#ifndef LV_UI_H
#define LV_UI_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 机器状态枚举（与 CAN 协议保持一致） */
#define UI_STATE_RUN    1   /* 运行 */
#define UI_STATE_IDLE   2   /* 待机 */
#define UI_STATE_STOP   0   /* 停止（其他值统一按 STOP 处理） */

/* 阀门状态枚举（(status>>2)&1） */
#define UI_VALVE_CLOSE  0   /* 关阀 */
#define UI_VALVE_OPEN   1   /* 开阀 */

/**
 * @brief 创建所有控件并初始化到默认值
 *        （转速 0、所有数据 0、状态 STOP/CLOSE、MQTT 断开）
 */
void ui_init(void);

/**
 * @brief 更新传感器数据卡（温度 / 湿度 / 烟雾）
 * @param temp   温度，整数.两位小数（如 25.34）
 * @param humi   湿度，整数.两位小数 %
 * @param smoke  烟雾，整数.两位小数 ppm
 */
void ui_update_sensor(float temp, float humi, float smoke);

/**
 * @brief 更新机器运行数据（转速仪表 + 中心大数字、压力卡、振动文字）
 * @param rpm       主轴转速 0~3000
 * @param pressure  压力，整数.两位小数 MPa
 * @param vib       振动，整数.两位小数 mm/s
 */
void ui_update_machine(int32_t rpm, float pressure, float vib);

/**
 * @brief 更新状态行（机器状态彩签、阀门彩签、按钮高亮边框）
 * @param state  machine state：1=RUN 2=IDLE 其他=STOP
 * @param valve  valve state：0=CLOSE 1=OPEN
 */
void ui_update_status(int32_t state, int32_t valve);

/**
 * @brief 更新右上角 MQTT 状态指示圆点
 * @param connected  true=已连接(绿)  false=断开(红)
 */
void ui_update_mqtt(bool connected);

/**
 * @brief 读取当前阀门状态（按钮设置值 / 最近一次 ui_update_status 传入值）
 * @return 0=CLOSE  1=OPEN
 */
int32_t ui_get_valve(void);

/**
 * @brief 阀门按钮回调（平台相关动作注入点，如发送 CAN 控制帧）
 * @param valve 0=CLOSE  1=OPEN
 * @note 由外部（如 main.c）调用 ui_set_valve_cb 注入; 未注入时按钮仅更新界面
 */
typedef void (*ui_valve_cb_t)(int32_t valve);
void ui_set_valve_cb(ui_valve_cb_t cb);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_UI_H*/
