/**
 * @file ui.c
 * @brief CAN->MQTT 机器监控网关 - 浅色工业 HMI 仪表盘（WinCC / Vijeo 风格）
 *        纯 LVGL 9.5 API，可原样移植到 STM32F407 + ST7789V（240x320 RGB565）
 *
 * 页面 1（主页，240x320 竖屏，绝对定位）：
 *   - 顶部标题栏     y=0..36    渐变灰底 + "F407 GATEWAY" + MQTT 状态 + TREND 导航
 *   - 仪表盘区域     y=36..140  半圆 lv_arc（0~3000 RPM，工业蓝）+ 28px 大数字
 *   - 数据卡片网格   y=146..244 2x2（105x46）：TEMP/HUMI/SMOKE/PRESS
 *   - 状态栏         y=246..272 上下边框 + RUN pill + VIB 数据 + CLOSE pill
 *   - 控制按钮       y=276..320 CLOSE/OPEN VALVE（渐变 + 按下位移反馈）
 *
 * 页面 2（传感器趋势页）：
 *   - 顶部标题栏     y=0..36    BACK 按钮 + "SENSOR TRENDS"
 *   - 三张折线图     TEMP/HUMI/SMOKE（lv_chart，各 224x70）
 *     横轴 = 最近 10 分钟（100 点 x 6s 采样，SHIFT 左移滚动）
 *   - 页脚           采样说明
 *
 * 移植说明：
 *   - 仅依赖 lvgl.h，无任何平台函数
 *   - ui_update_* 可被外部任务以 <=10ms 周期调用，内部无阻塞/延时
 *   - 28px 数值字体当前用内置 montserrat_28；Flash 紧张时可换成
 *     自定义"仅 0-9 和 ."字体（lvgl.io/tools/fontconverter 生成）
 */

#include "ui.h"

#include <stdio.h>  /* printf 仅用于模拟器阶段调试日志 */

/*********************
 *      颜色（RGB888，LVGL 自动转 RGB565）
 *********************/
#define COL_PAGE_BG      lv_color_hex(0xE8E4DF)  /* 页面底色：暖灰白 */
#define COL_HDR_GRAD1    lv_color_hex(0xD4D0CB)  /* 标题栏渐变起点 */
#define COL_HDR_GRAD2    lv_color_hex(0xC4C0BB)  /* 标题栏渐变终点 */
#define COL_HDR_BORDER   lv_color_hex(0x999999)  /* 标题栏底边框 */
#define COL_GAUGE_BG     lv_color_hex(0xF0ECE7)  /* 仪表盘区域底 */
#define COL_GAUGE_BORDER lv_color_hex(0xCCCCCC)  /* 仪表盘区域底边框 */
#define COL_STATUS_BG    lv_color_hex(0xDDD9D4)  /* 状态栏背景 */
#define COL_CARD_BG      lv_color_hex(0xF5F2EE)  /* 数据卡片背景 */
#define COL_CARD_BORDER  lv_color_hex(0xBBBBBB)  /* 卡片边框 */
#define COL_TRACK        lv_color_hex(0xD0CCC7)  /* 仪表盘底轨 */
#define COL_TITLE        lv_color_hex(0x333333)  /* 标题文字 */
#define COL_VALUE        lv_color_hex(0x222222)  /* 数值文字 */
#define COL_LABEL        lv_color_hex(0x666666)  /* 标签/单位文字 */
#define COL_IND_BLUE     lv_color_hex(0x1565C0)  /* 工业蓝：仪表/振动值 */
#define COL_MQTT_ON      lv_color_hex(0x2D7D32)  /* MQTT 在线文字 */
#define COL_WHITE        lv_color_hex(0xFFFFFF)  /* 按钮文字/高光 */

/* 状态 pill：绿（RUN/OPEN） */
#define COL_PILL_G_BG    lv_color_hex(0xC8E6C9)
#define COL_PILL_G_TXT   lv_color_hex(0x1B5E20)
#define COL_PILL_G_BORD  lv_color_hex(0x81C784)
/* 状态 pill：红（STOP/CLOSE） */
#define COL_PILL_R_BG    lv_color_hex(0xFFCDD2)
#define COL_PILL_R_TXT   lv_color_hex(0xB71C1C)
#define COL_PILL_R_BORD  lv_color_hex(0xEF5350)
/* 状态 pill：灰（IDLE） */
#define COL_PILL_N_BG    lv_color_hex(0xE3E0DB)
#define COL_PILL_N_TXT   lv_color_hex(0x555555)
#define COL_PILL_N_BORD  lv_color_hex(0xBBBBBB)

/* 按钮：红（CLOSE VALVE） */
#define COL_BTN_R_G1     lv_color_hex(0xEF5350)  /* 渐变起点 */
#define COL_BTN_R_G2     lv_color_hex(0xC62828)  /* 渐变终点 */
#define COL_BTN_R_BORD   lv_color_hex(0xB71C1C)
#define COL_BTN_R_PRESS  lv_color_hex(0xC62828)  /* 按下态纯色 */
/* 按钮：绿（OPEN VALVE） */
#define COL_BTN_G_G1     lv_color_hex(0x66BB6A)
#define COL_BTN_G_G2     lv_color_hex(0x2E7D32)
#define COL_BTN_G_BORD   lv_color_hex(0x1B5E20)
#define COL_BTN_G_PRESS  lv_color_hex(0x2E7D32)

/* 圆点符号 U+2022（内置 montserrat 字体已包含该字形，UTF-8 编码） */
#define STR_BULLET       "\xE2\x80\xA2"

/* ---------- 趋势页配置 ---------- */
#define TREND_POINT_CNT  100       /* 100 点 x 6s = 10 分钟窗口 */
#define TREND_PERIOD_MS  6000      /* 采样周期（毫秒） */

/**********************
 *      静态句柄
 **********************/

/* 页面 */
static lv_obj_t * s_scr_main;      /* 主页 screen（lv_screen_active() 默认屏） */
static lv_obj_t * s_scr_trend;     /* 趋势页 screen */

/* 顶部标题栏 */
static lv_obj_t * s_lbl_mqtt;      /* 右上角 MQTT 状态文字 */

/* 仪表盘 */
static lv_obj_t * s_arc;           /* 半圆转速仪表 */
static lv_obj_t * s_lbl_rpm;       /* 中心 28px 转速数字 */

/* 数据卡片数值 */
static lv_obj_t * s_lbl_temp;
static lv_obj_t * s_lbl_humi;
static lv_obj_t * s_lbl_smoke;
static lv_obj_t * s_lbl_press;

/* 状态栏 */
static lv_obj_t * s_pill_state;    /* 机器状态 pill */
static lv_obj_t * s_pill_state_lbl;
static lv_obj_t * s_lbl_vib_val;   /* 振动数值（蓝色） */
static lv_obj_t * s_pill_valve;    /* 阀门状态 pill */
static lv_obj_t * s_pill_valve_lbl;

/* 控制按钮 */
static lv_obj_t * s_btn_close;
static lv_obj_t * s_btn_open;

/* 当前阀门状态（按钮可改；F407 上由 CAN 实际状态覆盖） */
static int32_t s_ui_valve = UI_VALVE_CLOSE;

/* 阀门按钮注入回调（外部用 ui_set_valve_cb 设置，如发送 CAN 控制帧） */
static ui_valve_cb_t s_valve_cb = NULL;

/* 趋势页：图表 / 序列 / 当前值 */
static lv_obj_t * s_chart_temp;
static lv_obj_t * s_chart_humi;
static lv_obj_t * s_chart_smoke;
static lv_chart_series_t * s_ser_temp;
static lv_chart_series_t * s_ser_humi;
static lv_chart_series_t * s_ser_smoke;
static lv_obj_t * s_lbl_trend_temp;   /* 趋势页 TEMP 当前值 */
static lv_obj_t * s_lbl_trend_humi;
static lv_obj_t * s_lbl_trend_smoke;

/* 最近一次传感器值（采样定时器据此写入趋势图） */
static float s_last_temp = 0.0f;
static float s_last_humi = 0.0f;
static float s_last_smoke = 0.0f;

/**********************
 *  静态函数声明
 **********************/
static lv_obj_t * create_pill(lv_obj_t * parent, int32_t x, int32_t y, int32_t w);
static void set_pill_style(lv_obj_t * pill, lv_obj_t * lbl,
                           lv_color_t bg, lv_color_t txt, lv_color_t bord);
static void btn_close_cb(lv_event_t * e);
static void btn_open_cb(lv_event_t * e);
static void create_trend_page(void);
static void trend_chart_group(lv_obj_t * parent, int32_t y, const char * title,
                              const char * y_min_txt, const char * y_max_txt,
                              int32_t range_min, int32_t range_max, lv_color_t color,
                              lv_obj_t ** out_chart, lv_chart_series_t ** out_ser,
                              lv_obj_t ** out_val_lbl);
static void goto_trend_cb(lv_event_t * e);
static void goto_main_cb(lv_event_t * e);
static void trend_sample_cb(lv_timer_t * timer);

/**********************
 *  静态函数实现
 **********************/

/**
 * @brief 创建状态 pill 容器（自适应样式的圆角矩形 + 居中文字）
 * @return pill 对象（第一个子对象为文字 label）
 */
static lv_obj_t * create_pill(lv_obj_t * parent, int32_t x, int32_t y, int32_t w)
{
    lv_obj_t * pill = lv_obj_create(parent);
    lv_obj_remove_style_all(pill);
    lv_obj_set_pos(pill, x, y);
    lv_obj_set_size(pill, w, 20);
    lv_obj_set_style_radius(pill, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(pill, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pill, 0, LV_PART_MAIN);

    lv_obj_t * lbl = lv_label_create(pill);
    lv_obj_set_style_text_color(lbl, COL_PILL_G_TXT, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

    return pill;
}

/** @brief 一次性设置 pill 的背景/文字/边框配色 */
static void set_pill_style(lv_obj_t * pill, lv_obj_t * lbl,
                           lv_color_t bg, lv_color_t txt, lv_color_t bord)
{
    lv_obj_set_style_bg_color(pill, bg, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(pill, bord, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl, txt, 0);
}

/**
 * @brief "CLOSE VALVE" 按钮回调
 * 更新界面 + 触发注入的阀门回调（F407 上发送 CAN 关阀指令）
 */
static void btn_close_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    s_ui_valve = UI_VALVE_CLOSE;
    printf("[UI] CLOSE VALVE pressed -> valve=CLOSE\n");
    set_pill_style(s_pill_valve, s_pill_valve_lbl,
                   COL_PILL_R_BG, COL_PILL_R_TXT, COL_PILL_R_BORD);
    lv_label_set_text(s_pill_valve_lbl, STR_BULLET " CLOSE");
    if (s_valve_cb) s_valve_cb(UI_VALVE_CLOSE);
}

/**
 * @brief "OPEN VALVE" 按钮回调
 * 更新界面 + 触发注入的阀门回调（F407 上发送 CAN 开阀指令）
 */
static void btn_open_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    s_ui_valve = UI_VALVE_OPEN;
    printf("[UI] OPEN VALVE pressed -> valve=OPEN\n");
    set_pill_style(s_pill_valve, s_pill_valve_lbl,
                   COL_PILL_G_BG, COL_PILL_G_TXT, COL_PILL_G_BORD);
    lv_label_set_text(s_pill_valve_lbl, STR_BULLET " OPEN");
    if (s_valve_cb) s_valve_cb(UI_VALVE_OPEN);
}

/**********************
 *  趋势页实现
 **********************/

/**
 * @brief 主页 "TREND" 按钮回调：切换到传感器趋势页
 */
static void goto_trend_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    lv_screen_load_anim(s_scr_trend, LV_SCR_LOAD_ANIM_MOVE_LEFT, 150, 0, false);
}

/**
 * @brief 趋势页 "BACK" 按钮回调：返回主页
 */
static void goto_main_cb(lv_event_t * e)
{
    LV_UNUSED(e);
    lv_screen_load_anim(s_scr_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 150, 0, false);
}

/**
 * @brief 趋势采样定时器回调（lv_timer，周期 TREND_PERIOD_MS）
 *        把最近一次传感器值写入三张折线图（数值 x10 保留 1 位小数分辨率）。
 *        SHIFT 模式：数据满 100 点后自动左移，窗口恒为最近 10 分钟。
 */
static void trend_sample_cb(lv_timer_t * timer)
{
    LV_UNUSED(timer);
    lv_chart_set_next_value(s_chart_temp,  s_ser_temp,  (int32_t)(s_last_temp  * 10.0f));
    lv_chart_set_next_value(s_chart_humi,  s_ser_humi,  (int32_t)(s_last_humi  * 10.0f));
    lv_chart_set_next_value(s_chart_smoke, s_ser_smoke, (int32_t)(s_last_smoke * 10.0f));
}

/**
 * @brief 创建一个趋势图组：标题行（标签 + 当前值）+ 折线图 + Y 轴范围标签 + 时间轴标签
 * @param parent      父对象（趋势页 screen）
 * @param y           组的起始 y（标题行位置，图表在 y+12）
 * @param title       标题文本（如 "TEMP \xB0 C"）
 * @param y_min_txt   Y 轴最小值标签文本（如 "0"）
 * @param y_max_txt   Y 轴最大值标签文本（如 "60"）
 * @param range_min   Y 轴范围最小值（数值已 x10）
 * @param range_max   Y 轴范围最大值（数值已 x10）
 * @param color       折线颜色
 * @param out_chart   [出参] 图表对象
 * @param out_ser     [出参] 数据序列
 * @param out_val_lbl [出参] 当前值 label
 */
static void trend_chart_group(lv_obj_t * parent, int32_t y, const char * title,
                              const char * y_min_txt, const char * y_max_txt,
                              int32_t range_min, int32_t range_max, lv_color_t color,
                              lv_obj_t ** out_chart, lv_chart_series_t ** out_ser,
                              lv_obj_t ** out_val_lbl)
{
    /* ---- 标题行（y..y+12）：左标签 + 右当前值 ---- */
    lv_obj_t * lbl = lv_label_create(parent);
    lv_label_set_text(lbl, title);
    lv_obj_set_pos(lbl, 10, y);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_letter_space(lbl, 1, 0);

    lv_obj_t * val = lv_label_create(parent);
    lv_obj_set_pos(val, 168, y - 1);
    lv_obj_set_size(val, 62, 14);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(val, color, 0);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_12, 0);
    lv_label_set_text(val, "-");

    /* ---- 折线图（y+12 .. y+82，224x70） ---- */
    lv_obj_t * chart = lv_chart_create(parent);
    lv_obj_set_pos(chart, 8, y + 12);
    lv_obj_set_size(chart, 224, 70);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, TREND_POINT_CNT);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);   /* 满后左移滚动 */
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, range_min, range_max);
    lv_chart_set_div_line_count(chart, 3, 5);                      /* 3 横 5 竖网格 */

    /* 图表外观：白底、1px 灰边框、圆角 4 */
    lv_obj_set_style_bg_color(chart, COL_WHITE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(chart, COL_CARD_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(chart, 4, LV_PART_MAIN);
    /* 内边距：左 22 留给 Y 轴标签，底 12 留给时间轴标签 */
    lv_obj_set_style_pad_left(chart, 22, LV_PART_MAIN);
    lv_obj_set_style_pad_right(chart, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_top(chart, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(chart, 12, LV_PART_MAIN);
    /* 网格线：浅灰 1px */
    lv_obj_set_style_line_color(chart, lv_color_hex(0xE4E0DB), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
    /* 折线：2px 圆角端点；不画数据点 */
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_line_rounded(chart, true, LV_PART_ITEMS);
    lv_obj_set_style_width(chart, 0, LV_PART_INDICATOR);
    lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR);

    lv_chart_series_t * ser = lv_chart_add_series(chart, color, LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_values(chart, ser, LV_CHART_POINT_NONE);      /* 初始无数据 */

    /* ---- Y 轴范围标签（图表左内边距区，覆盖在图表上） ---- */
    int32_t cy = y + 12;
    lbl = lv_label_create(parent);
    lv_label_set_text(lbl, y_max_txt);
    lv_obj_set_pos(lbl, 10, cy + 1);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    lbl = lv_label_create(parent);
    lv_label_set_text(lbl, y_min_txt);
    lv_obj_set_pos(lbl, 10, cy + 47);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    /* ---- 时间轴标签（图表底部内边距区）：-10m / -5m / now ---- */
    lbl = lv_label_create(parent);
    lv_label_set_text(lbl, "-10m");
    lv_obj_set_pos(lbl, 30, cy + 58);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    lbl = lv_label_create(parent);
    lv_label_set_text(lbl, "-5m");
    lv_obj_set_pos(lbl, 112, cy + 58);
    lv_obj_set_size(lbl, 44, 12);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    lbl = lv_label_create(parent);
    lv_label_set_text(lbl, "now");
    lv_obj_set_pos(lbl, 198, cy + 58);
    lv_obj_set_size(lbl, 30, 12);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    *out_chart = chart;
    *out_ser = ser;
    *out_val_lbl = val;
}

/**
 * @brief 创建传感器趋势页（页面 2）并启动采样定时器
 */
static void create_trend_page(void)
{
    s_scr_trend = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_trend, COL_PAGE_BG, 0);
    lv_obj_set_style_bg_opa(s_scr_trend, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_scr_trend, 0, 0);
    lv_obj_set_style_radius(s_scr_trend, 0, 0);
    lv_obj_set_style_border_width(s_scr_trend, 0, 0);

    /* ---- 顶部标题栏（0..36，与主页同款渐变） ---- */
    lv_obj_t * header = lv_obj_create(s_scr_trend);
    lv_obj_remove_style_all(header);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_size(header, 240, 36);
    lv_obj_set_style_bg_color(header, COL_HDR_GRAD1, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(header, COL_HDR_GRAD2, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(header, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(header, COL_HDR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_side(header, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    /* BACK 按钮（浅灰工业按钮） */
    lv_obj_t * btn_back = lv_button_create(header);
    lv_obj_set_pos(btn_back, 8, 6);
    lv_obj_set_size(btn_back, 56, 24);
    lv_obj_set_style_radius(btn_back, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_back, COL_STATUS_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_back, COL_HDR_GRAD2, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(btn_back, COL_HDR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn_back, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn_back, 2, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(btn_back, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(btn_back, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btn_back, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn_back, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_back, goto_main_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, "BACK");
    lv_obj_set_style_text_color(back_lbl, COL_TITLE, 0);
    lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(back_lbl, 1, 0);
    lv_obj_center(back_lbl);

    /* 标题 */
    lv_obj_t * lbl = lv_label_create(header);
    lv_label_set_text(lbl, "SENSOR TRENDS");
    lv_obj_set_pos(lbl, 76, 12);
    lv_obj_set_style_text_color(lbl, COL_TITLE, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(lbl, 1, 0);

    /* ---- 三张折线图（TEMP / HUMI / SMOKE） ---- */
    trend_chart_group(s_scr_trend, 40, "TEMP \xC2\xB0" "C", "0", "60",
                      0, 600, lv_color_hex(0x1565C0),
                      &s_chart_temp, &s_ser_temp, &s_lbl_trend_temp);
    trend_chart_group(s_scr_trend, 126, "HUMI %", "0", "100",
                      0, 1000, lv_color_hex(0x2E7D32),
                      &s_chart_humi, &s_ser_humi, &s_lbl_trend_humi);
    trend_chart_group(s_scr_trend, 212, "SMOKE ppm", "0", "100",
                      0, 1000, lv_color_hex(0xE65100),
                      &s_chart_smoke, &s_ser_smoke, &s_lbl_trend_smoke);

    /* ---- 页脚采样说明 ---- */
    lbl = lv_label_create(s_scr_trend);
    lv_label_set_text(lbl, "LAST 10 MIN - 1 SAMPLE / 6 S");
    lv_obj_set_pos(lbl, 8, 296);
    lv_obj_set_size(lbl, 224, 12);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);

    /* ---- 采样定时器：每 6s 把最新传感器值写入折线图 ---- */
    lv_timer_create(trend_sample_cb, TREND_PERIOD_MS, NULL);
}

/**********************
 *   全局函数实现
 **********************/

void ui_init(void)
{
    /* ========== 0. 主页 screen（默认屏） ========== */
    lv_obj_t * scr = lv_screen_active();
    s_scr_main = scr;
    lv_obj_set_style_bg_color(scr, COL_PAGE_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_radius(scr, 0, 0);
    lv_obj_set_style_border_width(scr, 0, 0);

    /* ========== 1. 顶部标题栏（0..36，垂直渐变 + 底边框） ========== */
    lv_obj_t * header = lv_obj_create(scr);
    lv_obj_remove_style_all(header);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_size(header, 240, 36);
    lv_obj_set_style_bg_color(header, COL_HDR_GRAD1, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(header, COL_HDR_GRAD2, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(header, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(header, COL_HDR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_side(header, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    lv_obj_t * lbl_title = lv_label_create(header);
    lv_label_set_text(lbl_title, "F407 GATEWAY");
    lv_obj_set_pos(lbl_title, 12, 12);
    lv_obj_set_style_text_color(lbl_title, COL_TITLE, 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(lbl_title, 1, 0);   /* 全大写 + 1px 字距 */

    /* 右上角 MQTT 状态（默认离线，红色）；缩写 ON/OFF 才放得下且不与标题重叠 */
    s_lbl_mqtt = lv_label_create(header);
    lv_obj_set_pos(s_lbl_mqtt, 110, 13);
    lv_obj_set_size(s_lbl_mqtt, 68, 14);
    lv_label_set_long_mode(s_lbl_mqtt, LV_LABEL_LONG_CLIP);   /* 防止换行露出第二行残字 */
    lv_obj_set_style_text_align(s_lbl_mqtt, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(s_lbl_mqtt, COL_PILL_R_TXT, 0);
    lv_obj_set_style_text_font(s_lbl_mqtt, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_lbl_mqtt, STR_BULLET " MQTT OFF");

    /* TREND 导航按钮（切换到传感器趋势页） */
    lv_obj_t * btn_trend = lv_button_create(header);
    lv_obj_set_pos(btn_trend, 180, 6);
    lv_obj_set_size(btn_trend, 48, 24);
    lv_obj_set_style_radius(btn_trend, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_trend, COL_STATUS_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_trend, COL_HDR_GRAD2, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(btn_trend, COL_HDR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn_trend, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn_trend, 2, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(btn_trend, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(btn_trend, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btn_trend, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn_trend, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_trend, goto_trend_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * trend_lbl = lv_label_create(btn_trend);
    lv_label_set_text(trend_lbl, "TREND");
    lv_obj_set_style_text_color(trend_lbl, COL_TITLE, 0);
    lv_obj_set_style_text_font(trend_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(trend_lbl, 1, 0);
    lv_obj_center(trend_lbl);

    /* ========== 2. 仪表盘区域（36..140） ========== */
    lv_obj_t * gauge_sec = lv_obj_create(scr);
    lv_obj_remove_style_all(gauge_sec);
    lv_obj_set_pos(gauge_sec, 0, 36);
    lv_obj_set_size(gauge_sec, 240, 104);
    lv_obj_set_style_bg_color(gauge_sec, COL_GAUGE_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(gauge_sec, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(gauge_sec, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(gauge_sec, COL_GAUGE_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_side(gauge_sec, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);

    /* 禁止滚动：下方 arc 控件为正方形，包围盒超出区域但不绘制下半部分 */
    lv_obj_remove_flag(gauge_sec, LV_OBJ_FLAG_SCROLLABLE);

    /* 半圆仪表：控件必须是正方形（lv_arc 圆心取 min(w,h)/2 从左上角起算），
     * 160x160 于 (40,5)：半圆直径 160 跨 x=40..200 居中，弧带顶部 y=0，
     * 直径端点落在 y=80..90；角度 0..180 + 旋转 180 -> 只绘制顶部半圆，
     * 下半包围盒不画任何东西 */
    s_arc = lv_arc_create(gauge_sec);
    lv_obj_set_pos(s_arc, 40, 5);
    lv_obj_set_size(s_arc, 160, 160);
    lv_obj_remove_flag(s_arc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_arc, 0, LV_PART_MAIN);   /* 清零默认 padding，弧线才能水平居中 */
    lv_obj_set_style_pad_all(s_arc, 0, LV_PART_INDICATOR);
    lv_arc_set_rotation(s_arc, 180);
    lv_arc_set_bg_angles(s_arc, 0, 180);
    lv_arc_set_range(s_arc, 0, 3000);
    lv_arc_set_mode(s_arc, LV_ARC_MODE_NORMAL);
    lv_arc_set_value(s_arc, 0);
    lv_obj_remove_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);      /* 仅显示，不可拖动 */

    /* 底轨：轨道灰 10px 圆角端点 */
    lv_obj_set_style_arc_color(s_arc, COL_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(s_arc, true, LV_PART_MAIN);
    /* 进度条：工业蓝 10px 圆角端点（纯色，无渐变） */
    lv_obj_set_style_arc_color(s_arc, COL_IND_BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(s_arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_arc, true, LV_PART_INDICATOR);
    /* 隐藏默认旋钮 */
    lv_obj_set_style_bg_opa(s_arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_arc, 0, LV_PART_KNOB);

    /* 中心转速大数字（28px）+ 单位 */
    s_lbl_rpm = lv_label_create(gauge_sec);
    lv_obj_set_pos(s_lbl_rpm, 60, 44);
    lv_obj_set_size(s_lbl_rpm, 120, 30);
    lv_obj_set_style_text_align(s_lbl_rpm, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_lbl_rpm, COL_VALUE, 0);
    lv_obj_set_style_text_font(s_lbl_rpm, &lv_font_montserrat_28, 0);
    lv_label_set_text(s_lbl_rpm, "0");

    lv_obj_t * lbl_rpm_unit = lv_label_create(gauge_sec);
    lv_obj_set_pos(lbl_rpm_unit, 96, 76);
    lv_obj_set_size(lbl_rpm_unit, 48, 14);
    lv_obj_set_style_text_align(lbl_rpm_unit, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_rpm_unit, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl_rpm_unit, &lv_font_montserrat_12, 0);
    lv_label_set_text(lbl_rpm_unit, "RPM");

    /* 刻度标签：0 / 1500 / 3000（对准半圆左端 x=40 / 中点 x=120 / 右端 x=200） */
    lv_obj_t * lbl_tick;
    lbl_tick = lv_label_create(gauge_sec);
    lv_obj_set_pos(lbl_tick, 28, 92);
    lv_obj_set_size(lbl_tick, 24, 12);
    lv_obj_set_style_text_align(lbl_tick, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_tick, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl_tick, &lv_font_montserrat_10, 0);
    lv_label_set_text(lbl_tick, "0");

    lbl_tick = lv_label_create(gauge_sec);
    lv_obj_set_pos(lbl_tick, 96, 92);
    lv_obj_set_size(lbl_tick, 48, 12);
    lv_obj_set_style_text_align(lbl_tick, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_tick, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl_tick, &lv_font_montserrat_10, 0);
    lv_label_set_text(lbl_tick, "1500");

    lbl_tick = lv_label_create(gauge_sec);
    lv_obj_set_pos(lbl_tick, 188, 92);
    lv_obj_set_size(lbl_tick, 24, 12);
    lv_obj_set_style_text_align(lbl_tick, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_tick, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl_tick, &lv_font_montserrat_10, 0);
    lv_label_set_text(lbl_tick, "3000");

    /* ========== 3. 数据卡片网格（2x2，105x46，列/行距 6，边距 12） ========== */
    static const int32_t card_x[4] = {12, 123, 12, 123};   /* 左列/右列 */
    static const int32_t card_y[4] = {146, 146, 198, 198}; /* 上行/下行 */
    static const char * card_lbl_txt[4] = {"TEMP", "HUMI", "SMOKE", "PRESS"};
    lv_obj_t ** card_val[4] = {&s_lbl_temp, &s_lbl_humi, &s_lbl_smoke, &s_lbl_press};
    static const char * card_unit[4] = {"\xC2\xB0" "C", "%", "", ""};

    for(int i = 0; i < 4; i++) {
        lv_obj_t * card = lv_obj_create(scr);
        lv_obj_remove_style_all(card);
        lv_obj_set_pos(card, card_x[i], card_y[i]);
        lv_obj_set_size(card, 105, 46);
        lv_obj_set_style_radius(card, 4, LV_PART_MAIN);
        lv_obj_set_style_bg_color(card, COL_CARD_BG, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(card, COL_CARD_BORDER, LV_PART_MAIN);
        lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
        lv_obj_set_style_pad_all(card, 0, LV_PART_MAIN);
        /* 卡片外阴影（宽度 2、下移 1、20% 黑） */
        lv_obj_set_style_shadow_width(card, 2, LV_PART_MAIN);
        lv_obj_set_style_shadow_ofs_y(card, 1, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(card, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_shadow_opa(card, LV_OPA_20, LV_PART_MAIN);

        /* 顶部内高光：1px 白色 30% 细线（模拟卡片凸起） */
        lv_obj_t * hl = lv_obj_create(card);
        lv_obj_remove_style_all(hl);
        lv_obj_set_pos(hl, 1, 1);
        lv_obj_set_size(hl, 102, 1);
        lv_obj_set_style_bg_color(hl, COL_WHITE, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(hl, LV_OPA_30, LV_PART_MAIN);

        /* 标签（左上角，10px 全大写） */
        lv_obj_t * lbl = lv_label_create(card);
        lv_label_set_text(lbl, card_lbl_txt[i]);
        lv_obj_set_pos(lbl, 10, 6);
        lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_letter_space(lbl, 1, 0);

        /* 数值（标签下方 4px，16px 左对齐） */
        lv_obj_t * val = lv_label_create(card);
        lv_obj_set_pos(val, 10, 20);
        lv_obj_set_size(val, 50, 18);
        lv_obj_set_style_text_color(val, COL_VALUE, 0);
        lv_obj_set_style_text_font(val, &lv_font_montserrat_16, 0);
        lv_label_set_text(val, "-");
        *card_val[i] = val;

        /* 单位（紧跟数值右侧，基线大致对齐） */
        lv_obj_t * unit = lv_label_create(card);
        lv_label_set_text(unit, card_unit[i]);
        lv_obj_set_pos(unit, 62, 26);
        lv_obj_set_style_text_color(unit, COL_LABEL, 0);
        lv_obj_set_style_text_font(unit, &lv_font_montserrat_10, 0);
    }

    /* ========== 4. 状态栏（246..272，上下边框 + 三段内容） ========== */
    lv_obj_t * status = lv_obj_create(scr);
    lv_obj_remove_style_all(status);
    lv_obj_set_pos(status, 0, 246);
    lv_obj_set_size(status, 240, 26);
    lv_obj_set_style_bg_color(status, COL_STATUS_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(status, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(status, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(status, COL_CARD_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_side(status, LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_BOTTOM,
                                 LV_PART_MAIN);

    /* 左：机器状态 pill（默认 STOP 红） */
    s_pill_state = create_pill(status, 12, 3, 62);
    s_pill_state_lbl = lv_obj_get_child(s_pill_state, 0);
    set_pill_style(s_pill_state, s_pill_state_lbl,
                   COL_PILL_R_BG, COL_PILL_R_TXT, COL_PILL_R_BORD);
    lv_label_set_text(s_pill_state_lbl, STR_BULLET " STOP");

    /* 中：VIB 数据（"VIB" 灰 + 数值蓝 + "mm/s" 灰） */
    lv_obj_t * lbl = lv_label_create(status);
    lv_label_set_text(lbl, "VIB");
    lv_obj_set_pos(lbl, 78, 6);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x555555), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);

    s_lbl_vib_val = lv_label_create(status);
    lv_obj_set_pos(s_lbl_vib_val, 106, 6);
    lv_obj_set_size(s_lbl_vib_val, 26, 14);
    lv_obj_set_style_text_align(s_lbl_vib_val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(s_lbl_vib_val, COL_IND_BLUE, 0);
    lv_obj_set_style_text_font(s_lbl_vib_val, &lv_font_montserrat_12, 0);
    lv_label_set_text(s_lbl_vib_val, "0.0");

    lbl = lv_label_create(status);
    lv_label_set_text(lbl, "mm/s");
    lv_obj_set_pos(lbl, 136, 6);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x555555), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);

    /* 右：阀门状态 pill（默认 CLOSE 红） */
    s_pill_valve = create_pill(status, 164, 3, 64);
    s_pill_valve_lbl = lv_obj_get_child(s_pill_valve, 0);
    set_pill_style(s_pill_valve, s_pill_valve_lbl,
                   COL_PILL_R_BG, COL_PILL_R_TXT, COL_PILL_R_BORD);
    lv_label_set_text(s_pill_valve_lbl, STR_BULLET " CLOSE");

    /* ========== 5. 控制按钮区（276..320，104x44 x2，间距 8，边距 12） ========== */
    /* CLOSE VALVE：红渐变 */
    s_btn_close = lv_button_create(scr);
    lv_obj_set_pos(s_btn_close, 12, 276);
    lv_obj_set_size(s_btn_close, 104, 44);
    lv_obj_set_style_radius(s_btn_close, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_btn_close, COL_BTN_R_G1, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(s_btn_close, COL_BTN_R_G2, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(s_btn_close, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_btn_close, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_btn_close, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_btn_close, COL_BTN_R_BORD, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_btn_close, 3, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(s_btn_close, 2, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(s_btn_close, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(s_btn_close, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_btn_close, 0, LV_PART_MAIN);
    /* 按下态：纯色、去阴影、下移 1px（按钮"按下"物理感） */
    lv_obj_set_style_bg_color(s_btn_close, COL_BTN_R_PRESS, LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(s_btn_close, COL_BTN_R_PRESS, LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_dir(s_btn_close, LV_GRAD_DIR_NONE, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(s_btn_close, 0, LV_STATE_PRESSED);
    lv_obj_set_style_translate_y(s_btn_close, 1, LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_btn_close, btn_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_lbl = lv_label_create(s_btn_close);
    lv_label_set_text(btn_lbl, "CLOSE VALVE");
    lv_obj_set_style_text_color(btn_lbl, COL_WHITE, 0);
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(btn_lbl, 1, 0);
    lv_obj_center(btn_lbl);
    /* 按钮顶部内高光：1px 白 30% */
    lv_obj_t * btn_hl = lv_obj_create(s_btn_close);
    lv_obj_remove_style_all(btn_hl);
    lv_obj_set_pos(btn_hl, 2, 2);
    lv_obj_set_size(btn_hl, 98, 1);
    lv_obj_set_style_bg_color(btn_hl, COL_WHITE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_hl, LV_OPA_30, LV_PART_MAIN);

    /* OPEN VALVE：绿渐变 */
    s_btn_open = lv_button_create(scr);
    lv_obj_set_pos(s_btn_open, 124, 276);
    lv_obj_set_size(s_btn_open, 104, 44);
    lv_obj_set_style_radius(s_btn_open, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_btn_open, COL_BTN_G_G1, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(s_btn_open, COL_BTN_G_G2, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(s_btn_open, LV_GRAD_DIR_VER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_btn_open, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_btn_open, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_btn_open, COL_BTN_G_BORD, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_btn_open, 3, LV_PART_MAIN);
    lv_obj_set_style_shadow_ofs_y(s_btn_open, 2, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(s_btn_open, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(s_btn_open, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_btn_open, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_btn_open, COL_BTN_G_PRESS, LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(s_btn_open, COL_BTN_G_PRESS, LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_dir(s_btn_open, LV_GRAD_DIR_NONE, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(s_btn_open, 0, LV_STATE_PRESSED);
    lv_obj_set_style_translate_y(s_btn_open, 1, LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_btn_open, btn_open_cb, LV_EVENT_CLICKED, NULL);

    btn_lbl = lv_label_create(s_btn_open);
    lv_label_set_text(btn_lbl, "OPEN VALVE");
    lv_obj_set_style_text_color(btn_lbl, COL_WHITE, 0);
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_letter_space(btn_lbl, 1, 0);
    lv_obj_center(btn_lbl);
    btn_hl = lv_obj_create(s_btn_open);
    lv_obj_remove_style_all(btn_hl);
    lv_obj_set_pos(btn_hl, 2, 2);
    lv_obj_set_size(btn_hl, 98, 1);
    lv_obj_set_style_bg_color(btn_hl, COL_WHITE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_hl, LV_OPA_30, LV_PART_MAIN);

    /* ========== 创建趋势页（页面 2）并启动采样定时器 ========== */
    create_trend_page();

    /* ========== 初始化默认值（转速 0 / 数据 0 / STOP/CLOSE / MQTT 断开） ========== */
    ui_update_sensor(0.0f, 0.0f, 0.0f);
    ui_update_machine(0, 0.0f, 0.0f);
    ui_update_status(UI_STATE_STOP, UI_VALVE_CLOSE);
    ui_update_mqtt(false);
}

void ui_update_sensor(float temp, float humi, float smoke)
{
    lv_label_set_text_fmt(s_lbl_temp, "%.1f", (double)temp);    /* 如 25.1 */
    lv_label_set_text_fmt(s_lbl_humi, "%.1f", (double)humi);    /* 如 56.0 */
    lv_label_set_text_fmt(s_lbl_smoke, "%.1f", (double)smoke);  /* 如 9.0 */

    /* 记录最新值（趋势页采样定时器据此写入折线图），并刷新趋势页当前值 */
    s_last_temp = temp;
    s_last_humi = humi;
    s_last_smoke = smoke;
    lv_label_set_text_fmt(s_lbl_trend_temp, "%.1f", (double)temp);
    lv_label_set_text_fmt(s_lbl_trend_humi, "%.1f", (double)humi);
    lv_label_set_text_fmt(s_lbl_trend_smoke, "%.1f", (double)smoke);
}

void ui_update_machine(int32_t rpm, float pressure, float vib)
{
    /* 转速限幅 0~3000，防止越界 */
    if(rpm < 0) rpm = 0;
    if(rpm > 3000) rpm = 3000;

    lv_arc_set_value(s_arc, rpm);
    lv_label_set_text_fmt(s_lbl_rpm, "%d", rpm);
    lv_label_set_text_fmt(s_lbl_press, "%.2f", (double)pressure);     /* 如 0.53 */
    lv_label_set_text_fmt(s_lbl_vib_val, "%.1f", (double)vib);        /* 如 1.3 */
}

void ui_update_status(int32_t state, int32_t valve)
{
    /* 机器状态 pill：RUN=绿 / IDLE=灰 / 其他=STOP 红 */
    if(state == UI_STATE_RUN) {
        set_pill_style(s_pill_state, s_pill_state_lbl,
                       COL_PILL_G_BG, COL_PILL_G_TXT, COL_PILL_G_BORD);
        lv_label_set_text(s_pill_state_lbl, STR_BULLET " RUN");
    }
    else if(state == UI_STATE_IDLE) {
        set_pill_style(s_pill_state, s_pill_state_lbl,
                       COL_PILL_N_BG, COL_PILL_N_TXT, COL_PILL_N_BORD);
        lv_label_set_text(s_pill_state_lbl, STR_BULLET " IDLE");
    }
    else {
        set_pill_style(s_pill_state, s_pill_state_lbl,
                       COL_PILL_R_BG, COL_PILL_R_TXT, COL_PILL_R_BORD);
        lv_label_set_text(s_pill_state_lbl, STR_BULLET " STOP");
    }

    /* 阀门 pill：OPEN=绿 / CLOSE=红 */
    if(valve == UI_VALVE_OPEN) {
        set_pill_style(s_pill_valve, s_pill_valve_lbl,
                       COL_PILL_G_BG, COL_PILL_G_TXT, COL_PILL_G_BORD);
        lv_label_set_text(s_pill_valve_lbl, STR_BULLET " OPEN");
    }
    else {
        set_pill_style(s_pill_valve, s_pill_valve_lbl,
                       COL_PILL_R_BG, COL_PILL_R_TXT, COL_PILL_R_BORD);
        lv_label_set_text(s_pill_valve_lbl, STR_BULLET " CLOSE");
    }
}

void ui_update_mqtt(bool connected)
{
    /* 在线=绿 "ON"，断开=红 "OFF"（缩写，标题栏空间有限） */
    lv_obj_set_style_text_color(s_lbl_mqtt, connected ? COL_MQTT_ON : COL_PILL_R_TXT, 0);
    lv_label_set_text(s_lbl_mqtt, connected ? STR_BULLET " MQTT ON"
                                            : STR_BULLET " MQTT OFF");
}

int32_t ui_get_valve(void)
{
    return s_ui_valve;
}

void ui_set_valve_cb(ui_valve_cb_t cb)
{
    s_valve_cb = cb;
}
