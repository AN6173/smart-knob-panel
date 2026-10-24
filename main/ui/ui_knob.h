#pragma once

/**
 * @file ui_knob.h
 * @brief 旋钮屏 UI（无触摸）：旋转 = 移动/调节，GPIO14 短按 = 进入/确认，长按 = 返回/急停。
 */

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_EVT_SHORT = 0,
    UI_EVT_LONG,
} ui_evt_t;

/** 固件入口：初始化屏幕 + LVGL（core 0 低优先级）并创建界面 */
void ui_knob_start(void);

/** 仅创建界面（不碰硬件）；PC 预览程序也调用它 */
void ui_knob_create(lv_obj_t *scr);

/** 按键事件（任意任务可调用；单生产者无锁环形队列，由 LVGL 任务消费） */
void ui_knob_post(ui_evt_t evt);

#ifdef __cplusplus
}
#endif
