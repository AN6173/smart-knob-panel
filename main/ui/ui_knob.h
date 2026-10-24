#pragma once

/**
 * @file ui_knob.h
 * @brief 旋钮屏 UI（无触摸）：旋转 = 移动/调节，GPIO14 短按 = 进入/确认，长按 = 返回/急停。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_EVT_SHORT = 0,
    UI_EVT_LONG,
} ui_evt_t;

/** 初始化屏幕 + LVGL + 界面；LVGL 跑在 core 0 低优先级 */
void ui_knob_start(void);

/** 按键事件（任意任务可调用，内部入队，由 LVGL 任务处理） */
void ui_knob_post(ui_evt_t evt);

#ifdef __cplusplus
}
#endif
