/**
 * @file lv_port.c
 * @brief LVGL 移植层 - ST7789V 显示 / XPT2046 触摸 / 节拍
 *        对接正点原子 F407 探索者标准库驱动 (lcd.c / touch.c)
 *        用于 FreeRTOS: app_task 每 10ms 调 lv_timer_handler()
 */

#include "lvgl.h"
#include "lcd.h"
#include "touch.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

/**********************
 *  LCD flush 回调
 *  LVGL 渲染 RGB565 像素到 px_map, 需拷贝到 [area] 对应屏幕区域
 **********************/
static void disp_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint16_t *p = (uint16_t *)px_map;
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    int32_t x, y;

    /* 逐行写, 每行先设窗口再写 GRAM */
    for (y = 0; y < h; y++)
    {
        LCD_Set_Window(area->x1, area->y1 + y, w, 1);   /* 设 1 行窗口 */
        LCD_WriteRAM_Prepare();                           /* 准备写 GRAM */
        for (x = 0; x < w; x++)
        {
            LCD_WriteRAM(p[y * w + x]);
        }
    }

    /* 通知 LVGL flush 完成 (必须) */
    lv_display_flush_ready(disp);
}

/**********************
 *  触摸 read 回调
 **********************/
/* 诊断: 最近一次按下的像素坐标 (app_task 读取显示, 用于校准偏差排查) */
volatile int16_t g_touch_x = -1;
volatile int16_t g_touch_y = -1;
volatile uint8_t g_touch_active = 0;

static void indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static lv_point_t last_dw = { 120, 160 };   /* 上次按下点, 初始为屏幕内有效点, 避免未触摸时 LVGL 报 X/Y=-1 警告刷屏 */
    uint16_t tx, ty;

    if (TP_Read(&tx, &ty))    /* 返回 1 = 正在按下, 给出屏幕像素坐标 */
    {
        data->point.x = tx;
        data->point.y = ty;
        data->state  = LV_INDEV_STATE_PRESSED;
        last_dw.x = tx;
        last_dw.y = ty;
        g_touch_x = tx;  g_touch_y = ty;  g_touch_active = 1;
    }
    else                      /* 未按下 */
    {
        data->point.x = last_dw.x;
        data->point.y = last_dw.y;
        data->state  = LV_INDEV_STATE_RELEASED;
        g_touch_active = 0;
    }
    (void)indev;
}

/**********************
 *  节拍 (FreeRTOS tick)
 **********************/
#if LV_USE_OS == LV_OS_NONE
static uint32_t lv_tick_get_ms(void)
{
    return (uint32_t)xTaskGetTickCount();
}
#endif

/**********************
 *  初始化入口
 **********************/
void lv_port_init(void)
{
    static lv_color_t buf1[240 * 16];    /* 部分帧缓冲: 240 x 16 像素 (节省RAM) */

    lv_display_t *disp = lv_display_create(240, 320);
    lv_display_set_flush_cb(disp, disp_flush_cb);
    lv_display_set_buffers(disp, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, indev_read_cb);

#if LV_USE_OS == LV_OS_NONE
    lv_tick_set_cb(lv_tick_get_ms);     /* 用 FreeRTOS tick 做节拍 */
#endif
}