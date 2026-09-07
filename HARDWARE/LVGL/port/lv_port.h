/**
 * @file lv_port.h
 * @brief LVGL 移植层接口
 */
#ifndef LV_PORT_H
#define LV_PORT_H

#include <stdint.h>

void lv_port_init(void);

/* 诊断: 最近一次触摸的像素坐标与按下状态 (lv_port.c 定义) */
extern volatile int16_t g_touch_x;
extern volatile int16_t g_touch_y;
extern volatile uint8_t g_touch_active;

#endif /* LV_PORT_H */