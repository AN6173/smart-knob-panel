#pragma once

/**
 * @file lcd_tdisplay.h
 * @brief LILYGO T-Display-S3 板载屏：1.9" ST7789 170×320，8 位 i80 并口；LVGL 8.3 移植。
 *
 * 引脚（官方 pin_config.h）：D0–D7 = 39,40,41,42,45,46,47,48；WR 8；RD 9；DC 7；CS 6；
 * RST 5；背光 38；LCD 供电使能 15（电池供电时必须拉高）。与电机/编码器引脚无冲突。
 */

#include <stdbool.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_H_RES 320
#define LCD_V_RES 170

/** 初始化屏幕 + LVGL，并在指定核上启动 LVGL 任务（低优先级） */
bool lcd_tdisplay_start(int core, int priority);

/** LVGL 非线程安全：其它任务调用 lv_* 前后必须加锁 */
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);

/** 背光亮度 0–100 */
void lcd_set_backlight(int percent);

#ifdef __cplusplus
}
#endif
