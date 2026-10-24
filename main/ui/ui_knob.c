/**
 * @file ui_knob.c
 * @brief 旋钮屏 UI（LVGL 8.3，320×170 横屏，与 AMOLED 面板同一套深色香槟金 + 深青风格）。
 *
 * 页面：
 *   菜单：五种模式横向轮播（自由 / 卡位 / 开关 / 调光 / 窗帘），旋钮 30°/格带咔哒
 *   模式：左侧数值圆环实时跟随旋钮，右侧模式名 + 详情
 *   状态栏：驱动状态（运行 / 休眠 / 驱动未就绪）+ 估算电流 + 估算温升
 *
 * 所有界面逻辑在 LVGL 定时器里执行（即 lvgl 任务内），无需额外加锁；
 * 按键事件经队列投递。驱动未就绪时仍用编码器角度导航（不出力矩）。
 */

#include "ui_knob.h"

#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lvgl.h"

#include "../foc_task.h"
#include "lcd_tdisplay.h"
#include "fonts/ui_fonts.h"

static const char *TAG = "ui";

/* ---------- 配色（与 waveshare-amoled-neon-ui 一致） ---------- */
#define C_BG     lv_color_hex(0x0B0C0E)
#define C_CARD   lv_color_hex(0x16171A)
#define C_GOLD   lv_color_hex(0xE8C89A)
#define C_TEAL   lv_color_hex(0x2BB3A6)
#define C_RED    lv_color_hex(0xE5484D)
#define C_TXT1   lv_color_hex(0xF5EFE6)
#define C_TXT2   lv_color_hex(0x9A9387)
#define C_TXT3   lv_color_hex(0x5E5950)
#define C_TRACK  lv_color_hex(0x24231F)

/* 估算温升：绕组+外壳对环境热阻（2804 经验值），乘以 I²t 热模型给出的铜损 */
#define RTH_K_PER_W 25.0f

typedef struct {
    haptic_mode_t mode;
    const char *name;
    const char *desc;
} mode_item_t;

static const mode_item_t k_modes[] = {
    {HAPTIC_FREE, "自由", "无阻尼 · 角度"},
    {HAPTIC_DETENT, "卡位", "12 格 · 咔哒"},
    {HAPTIC_SWITCH, "开关", "双稳态"},
    {HAPTIC_DIMMER, "调光", "0–100% · 软端点"},
    {HAPTIC_CURTAIN, "窗帘", "20 格 · 端点"},
};
#define MODE_N 5

typedef enum { PAGE_MENU = 0, PAGE_MODE } page_t;

static QueueHandle_t s_evtq;
static page_t s_page = PAGE_MENU;
static int s_sel = 0;
static int32_t s_saved_pos[MODE_N];
static int32_t s_last_pos = INT32_MIN;
static int s_last_sel = -1;

/* 控件 */
static lv_obj_t *s_st_dot, *s_st_txt, *s_st_right;
static lv_obj_t *s_menu, *s_m_title, *s_m_desc, *s_m_prev, *s_m_next, *s_m_dots[MODE_N];
static lv_obj_t *s_mode, *s_arc, *s_val, *s_unit, *s_big, *s_title, *s_sub, *s_hint;
static lv_obj_t *s_toast;

static lv_obj_t *mk_label(lv_obj_t *p, const lv_font_t *f, lv_color_t c, const char *t)
{
    lv_obj_t *l = lv_label_create(p);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    lv_label_set_text(l, t);
    return l;
}

static lv_obj_t *mk_panel(lv_obj_t *p)
{
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, LCD_H_RES, LCD_V_RES - 24);
    lv_obj_set_pos(o, 0, 24);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

static void anim_opa_cb(void *o, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)o, (lv_opa_t)v, 0);
}

static void toast(const char *txt)
{
    lv_label_set_text(s_toast, txt);
    lv_obj_clear_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_opa(s_toast, LV_OPA_COVER, 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_toast);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_delay(&a, 900);
    lv_anim_set_time(&a, 400);
    lv_anim_set_exec_cb(&a, anim_opa_cb);
    lv_anim_start(&a);
}

/* ---------- 构建界面 ---------- */

static void build_status(lv_obj_t *scr)
{
    s_st_dot = lv_obj_create(scr);
    lv_obj_remove_style_all(s_st_dot);
    lv_obj_set_size(s_st_dot, 8, 8);
    lv_obj_set_style_radius(s_st_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_st_dot, LV_OPA_COVER, 0);
    lv_obj_set_pos(s_st_dot, 10, 8);
    s_st_txt = mk_label(scr, &font_cn_16, C_TXT2, "");
    lv_obj_set_pos(s_st_txt, 24, 2);
    s_st_right = mk_label(scr, &font_cn_16, C_TXT2, "");
    lv_obj_align(s_st_right, LV_ALIGN_TOP_RIGHT, -10, 2);
    lv_obj_t *line = lv_obj_create(scr);
    lv_obj_remove_style_all(line);
    lv_obj_set_size(line, LCD_H_RES - 20, 1);
    lv_obj_set_pos(line, 10, 23);
    lv_obj_set_style_bg_color(line, C_TRACK, 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
}

static void build_menu(lv_obj_t *scr)
{
    s_menu = mk_panel(scr);
    s_m_title = mk_label(s_menu, &font_cn_36, C_GOLD, "");
    lv_obj_align(s_m_title, LV_ALIGN_TOP_MID, 0, 22);
    s_m_desc = mk_label(s_menu, &font_cn_16, C_TXT2, "");
    lv_obj_align(s_m_desc, LV_ALIGN_TOP_MID, 0, 72);
    s_m_prev = mk_label(s_menu, &font_cn_22, C_TXT3, "");
    lv_obj_align(s_m_prev, LV_ALIGN_TOP_LEFT, 16, 32);
    s_m_next = mk_label(s_menu, &font_cn_22, C_TXT3, "");
    lv_obj_align(s_m_next, LV_ALIGN_TOP_RIGHT, -16, 32);
    for (int i = 0; i < MODE_N; i++) {
        lv_obj_t *d = lv_obj_create(s_menu);
        lv_obj_remove_style_all(d);
        lv_obj_set_size(d, 6, 6);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_align(d, LV_ALIGN_TOP_MID, (i - 2) * 16, 100);
        s_m_dots[i] = d;
    }
    lv_obj_t *h = mk_label(s_menu, &font_cn_16, C_TXT3, "旋转选择 · 短按进入 · 长按急停");
    lv_obj_align(h, LV_ALIGN_BOTTOM_MID, 0, -4);
}

static void build_mode(lv_obj_t *scr)
{
    s_mode = mk_panel(scr);
    s_arc = lv_arc_create(s_mode);
    lv_obj_set_size(s_arc, 132, 132);
    lv_obj_set_pos(s_arc, 10, 6);
    lv_arc_set_rotation(s_arc, 135);
    lv_arc_set_bg_angles(s_arc, 0, 270);
    lv_arc_set_range(s_arc, 0, 1000);
    lv_obj_remove_style(s_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc, C_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc, C_GOLD, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_arc, true, LV_PART_INDICATOR);

    s_val = mk_label(s_mode, &font_num_48, C_TXT1, "0");
    lv_obj_align_to(s_val, s_arc, LV_ALIGN_CENTER, 0, -4);
    s_big = mk_label(s_mode, &font_cn_36, C_TXT1, "");
    lv_obj_align_to(s_big, s_arc, LV_ALIGN_CENTER, 0, -4);
    s_unit = mk_label(s_mode, &font_cn_16, C_TXT2, "");
    lv_obj_align_to(s_unit, s_arc, LV_ALIGN_CENTER, 0, 30);

    s_title = mk_label(s_mode, &font_cn_36, C_GOLD, "");
    lv_obj_set_pos(s_title, 162, 14);
    s_sub = mk_label(s_mode, &font_cn_22, C_TXT1, "");
    lv_obj_set_pos(s_sub, 162, 62);
    s_hint = mk_label(s_mode, &font_cn_16, C_TXT3, "短按确认 · 长按返回");
    lv_obj_set_pos(s_hint, 162, 120);
}

/* ---------- 刷新 ---------- */

static void enter_menu(void)
{
    s_page = PAGE_MENU;
    /* 先切控制路径（它会重置原点），再按选中项设定格位 */
    if (foc_is_ok() && foc_get_ctrl_mode() != FOC_CTRL_HAPTIC) {
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    }
    foc_set_haptic_mode_pos(HAPTIC_MENU, s_sel);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    s_last_sel = -1;
    ESP_LOGI(TAG, "page=menu sel=%d %s", s_sel, k_modes[s_sel].name);
}

static void enter_mode(void)
{
    s_page = PAGE_MODE;
    const mode_item_t *m = &k_modes[s_sel];
    if (foc_is_ok() && foc_get_ctrl_mode() != FOC_CTRL_HAPTIC) {
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    }
    foc_set_haptic_mode_pos(m->mode, s_saved_pos[s_sel]);
    lv_label_set_text(s_title, m->name);
    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    s_last_pos = INT32_MIN;
    ESP_LOGI(TAG, "page=mode %s pos=%ld", m->name, (long)s_saved_pos[s_sel]);
}

static void refresh_status(void)
{
    foc_power_t p = {};
    foc_get_power(&p);
    const bool ok = foc_is_ok();
    const foc_ctrl_mode_t c = foc_get_ctrl_mode();
    lv_color_t col;
    const char *txt;
    if (!foc_is_ready()) {
        col = C_GOLD;
        txt = "校准中";
    } else if (!ok) {
        col = C_RED;
        txt = "驱动未就绪 · 无力反馈";
    } else if (c == FOC_CTRL_SENSOR) {
        col = C_TXT3;
        txt = "已停止";
    } else if (p.sleeping) {
        col = C_TEAL;
        txt = "休眠";
    } else {
        col = C_GOLD;
        txt = "运行";
    }
    lv_obj_set_style_bg_color(s_st_dot, col, 0);
    lv_label_set_text(s_st_txt, txt);
    lv_obj_set_style_text_color(s_st_txt, ok ? C_TXT2 : C_RED, 0);
    lv_label_set_text_fmt(s_st_right, "%.2fA  温升≈%.1f°C", fabsf(p.i_est),
                          (double)(p.p_copper * RTH_K_PER_W));
    lv_obj_align(s_st_right, LV_ALIGN_TOP_RIGHT, -10, 2);
}

static void refresh_menu(const foc_haptic_snapshot_t *hs)
{
    if (hs->mode == HAPTIC_MENU) {
        s_sel = (int)hs->position;
    }
    if (s_sel == s_last_sel) {
        return;
    }
    s_last_sel = s_sel;
    lv_label_set_text(s_m_title, k_modes[s_sel].name);
    lv_obj_align(s_m_title, LV_ALIGN_TOP_MID, 0, 22);
    lv_label_set_text(s_m_desc, k_modes[s_sel].desc);
    lv_obj_align(s_m_desc, LV_ALIGN_TOP_MID, 0, 72);
    lv_label_set_text(s_m_prev, s_sel > 0 ? k_modes[s_sel - 1].name : "");
    lv_label_set_text(s_m_next, s_sel < MODE_N - 1 ? k_modes[s_sel + 1].name : "");
    lv_obj_align(s_m_next, LV_ALIGN_TOP_RIGHT, -16, 32);
    for (int i = 0; i < MODE_N; i++) {
        lv_obj_set_style_bg_color(s_m_dots[i], i == s_sel ? C_GOLD : C_TRACK, 0);
        lv_obj_set_size(s_m_dots[i], i == s_sel ? 14 : 6, 6);
    }
    printf("[ui] menu sel=%d %s\n", s_sel, k_modes[s_sel].name);
}

static void show_number(bool num)
{
    if (num) {
        lv_obj_clear_flag(s_val, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_big, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_val, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_big, LV_OBJ_FLAG_HIDDEN);
    }
}

static void refresh_mode(const foc_haptic_snapshot_t *hs)
{
    const mode_item_t *m = &k_modes[s_sel];
    if (hs->mode != m->mode) {
        return;
    }
    const float deg = hs->shaft_rad * 57.29578f;
    int arc = 0;
    bool changed = (hs->position != s_last_pos);

    switch (m->mode) {
    case HAPTIC_FREE: {
        float a = fmodf(deg, 360.0f);
        if (a < 0) {
            a += 360.0f;
        }
        arc = (int)(a / 360.0f * 1000.0f);
        show_number(true);
        lv_label_set_text_fmt(s_val, "%d", (int)a);
        lv_label_set_text(s_unit, "°");
        lv_label_set_text_fmt(s_sub, "圈数 %+.2f", (double)(deg / 360.0f));
        lv_obj_set_style_arc_color(s_arc, C_TEAL, LV_PART_INDICATOR);
        break;
    }
    case HAPTIC_DETENT: {
        int32_t idx = hs->position % 12;
        if (idx < 0) {
            idx += 12;
        }
        arc = (int)(idx * 1000 / 11);
        show_number(true);
        lv_label_set_text_fmt(s_val, "%ld", (long)idx);
        lv_label_set_text(s_unit, "档");
        lv_label_set_text_fmt(s_sub, "累计 %+ld 格", (long)hs->position);
        lv_obj_set_style_arc_color(s_arc, C_GOLD, LV_PART_INDICATOR);
        break;
    }
    case HAPTIC_SWITCH: {
        const bool on = hs->position > 0;
        arc = on ? 1000 : 0;
        show_number(false);
        lv_label_set_text(s_big, on ? "开" : "关");
        lv_obj_set_style_text_color(s_big, on ? C_GOLD : C_TXT2, 0);
        lv_label_set_text(s_unit, "");
        lv_label_set_text(s_sub, on ? "灯已打开" : "灯已关闭");
        lv_obj_set_style_arc_color(s_arc, on ? C_GOLD : C_TXT3, LV_PART_INDICATOR);
        break;
    }
    case HAPTIC_DIMMER:
    case HAPTIC_CURTAIN: {
        const int pct = (int)lroundf(hs->value_norm * 100.0f);
        arc = pct * 10;
        show_number(true);
        lv_label_set_text_fmt(s_val, "%d", pct);
        lv_label_set_text(s_unit, "%");
        if (m->mode == HAPTIC_DIMMER) {
            lv_label_set_text_fmt(s_sub, "亮度 %d%%", pct);
            lv_obj_set_style_arc_color(s_arc, C_GOLD, LV_PART_INDICATOR);
        } else {
            lv_label_set_text_fmt(s_sub, "开合 %d%%", pct);
            lv_obj_set_style_arc_color(s_arc, C_TEAL, LV_PART_INDICATOR);
        }
        break;
    }
    default:
        break;
    }
    lv_arc_set_value(s_arc, arc);
    lv_obj_align_to(s_val, s_arc, LV_ALIGN_CENTER, 0, -4);
    lv_obj_align_to(s_big, s_arc, LV_ALIGN_CENTER, 0, -4);
    lv_obj_align_to(s_unit, s_arc, LV_ALIGN_CENTER, 0, 30);

    if (changed) {
        s_last_pos = hs->position;
        s_saved_pos[s_sel] = hs->position;
        printf("[ui] %s pos=%ld value=%.0f%% angle=%.1f°\n", m->name, (long)hs->position,
               (double)(hs->value_norm * 100.0f), (double)deg);
    }
}

static void handle_event(ui_evt_t e)
{
    if (e == UI_EVT_SHORT) {
        if (s_page == PAGE_MENU) {
            enter_mode();
        } else if (k_modes[s_sel].mode == HAPTIC_SWITCH) {
            const int32_t np = s_saved_pos[s_sel] > 0 ? 0 : 1;
            s_saved_pos[s_sel] = np;
            foc_set_haptic_mode_pos(HAPTIC_SWITCH, np);
            printf("[ui] switch toggle -> %s\n", np ? "开" : "关");
        } else {
            toast("已确认");
            printf("[ui] confirm %s pos=%ld\n", k_modes[s_sel].name, (long)s_saved_pos[s_sel]);
        }
    } else {
        if (s_page == PAGE_MODE) {
            enter_menu();
        } else {
            foc_stop();
            toast("急停");
            printf("[ui] e-stop\n");
        }
    }
}

static void ui_timer_cb(lv_timer_t *t)
{
    ui_evt_t e;
    while (xQueueReceive(s_evtq, &e, 0) == pdTRUE) {
        handle_event(e);
    }
    foc_haptic_snapshot_t hs;
    foc_get_haptic_snapshot(&hs);
    if (s_page == PAGE_MENU) {
        refresh_menu(&hs);
    } else {
        refresh_mode(&hs);
    }
    static uint32_t n = 0;
    if ((n++ % 5) == 0) {
        refresh_status();
    }
}

/* 启动后等 FOC 初始化完成再接管触感模式 */
static void ui_boot_cb(lv_timer_t *t)
{
    if (!foc_is_ready()) {
        refresh_status();
        return;
    }
    lv_timer_del(t);
    enter_menu();
    lv_timer_create(ui_timer_cb, 30, NULL);
}

void ui_knob_post(ui_evt_t evt)
{
    if (s_evtq) {
        xQueueSend(s_evtq, &evt, 0);
    }
}

void ui_knob_start(void)
{
    s_evtq = xQueueCreate(8, sizeof(ui_evt_t));
    if (!lcd_tdisplay_start(0, 2)) {
        ESP_LOGE(TAG, "display init failed — UI disabled");
        return;
    }
    lvgl_lock(-1);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    build_status(scr);
    build_menu(scr);
    build_mode(scr);
    lv_obj_add_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_m_title, "Smart Knob");
    lv_obj_align(s_m_title, LV_ALIGN_TOP_MID, 0, 22);

    s_toast = mk_label(scr, &font_cn_22, C_TXT1, "");
    lv_obj_set_style_bg_color(s_toast, C_CARD, 0);
    lv_obj_set_style_bg_opa(s_toast, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_toast, 12, 0);
    lv_obj_set_style_pad_hor(s_toast, 14, 0);
    lv_obj_set_style_pad_ver(s_toast, 6, 0);
    lv_obj_align(s_toast, LV_ALIGN_CENTER, 0, 10);
    lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);

    lv_timer_create(ui_boot_cb, 100, NULL);
    lvgl_unlock();
    lcd_set_backlight(80);
    ESP_LOGI(TAG, "UI started");
}
