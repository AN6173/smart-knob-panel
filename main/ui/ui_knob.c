/**
 * @file ui_knob.c
 * @brief 旋钮屏 UI（LVGL 8.3，320×170 横屏）—— 与 waveshare-amoled-neon-ui 同一套质感语言：
 *        香槟金 + 深青、预渲染暗角/颗粒背景、玻璃圆盘与卡片、Inter + 思源黑体 + Tabler 线性图标。
 *
 * 页面：
 *   菜单：五种模式横向轮播。轮播位置 = 当前格位 + 格内偏移（连续量），
 *         每帧缓动逼近 → 拧动时跟手、过格时与电机「咔哒」同步吸附。
 *   详情：左侧渐变圆环 + 刻度（每格一刻）+ 端点标记，数值实时跟随；右侧玻璃卡片显示名称/详情。
 *   状态栏：驱动状态（运行 / 休眠 / 已停止 / 驱动未就绪）、估算电流、估算温升。
 *
 * 线程：全部界面逻辑在 LVGL 定时器（lvgl 任务）中执行；按键经无锁环形队列投递。
 * 驱动未就绪时仍按编码器角度导航（无力反馈）。
 */

#include "ui_knob.h"

#include <math.h>
#include <stdio.h>

#include "../foc_task.h"
#include "assets/ui_assets.h"
#include "fonts/ui_fonts.h"

#define SCR_W 320
#define SCR_H 170

/* ---------- 配色（与 AMOLED 项目一致） ---------- */
#define C_GOLD   lv_color_hex(0xE8C89A)
#define C_GOLD_HI lv_color_hex(0xF6E3C2)
#define C_TEAL   lv_color_hex(0x2BB3A6)
#define C_RED    lv_color_hex(0xE5735F)
#define C_TXT1   lv_color_hex(0xF5EFE6)
#define C_TXT2   lv_color_hex(0x9A9387)
#define C_TXT3   lv_color_hex(0x5E5950)
#define C_TRACK  lv_color_hex(0x23211E)

/* Tabler Icons 码位（与 tools/gen_fonts.sh 同步） */
#define IC_FREE    "\xEE\xBE\x85" /* rotate-360  EF85 */
#define IC_DETENT  "\xEE\xBE\xB1" /* circle-dot  EFB1 */
#define IC_SWITCH  "\xEE\xAC\xBF" /* toggle-right EB3F */
#define IC_DIMMER  "\xEE\xA9\x91" /* bulb        EA51 */
#define IC_CURTAIN "\xEE\xBC\x86" /* window      EF06 */
#define IC_BOLT    "\xEE\xA8\xB8" /* bolt        EA38 */
#define IC_MOON    "\xEE\xAB\xB8" /* moon        EAF8 */
#define IC_ALERT   "\xEE\xA8\x86" /* alert-triangle EA06 */
#define IC_TEMP    "\xEE\xAC\xB8" /* temperature EB38 */
#define IC_STOP    "\xEE\xB5\x8A" /* player-stop ED4A */
#define IC_ACT     "\xEE\xB4\xA3" /* activity    ED23 */

/* 估算温升：铜损 × 绕组到环境热阻（2804 经验值） */
#define RTH_K_PER_W 25.0f

typedef struct {
    haptic_mode_t mode;
    const char *icon;
    const char *name;
    const char *desc;
    int ticks;      /* 圆环刻度数 */
    bool bounded;   /* 是否有端点 */
} mode_item_t;

static const mode_item_t k_modes[] = {
    {HAPTIC_FREE, IC_FREE, "自由", "无阻尼  ·  连续旋转", 36, false},
    {HAPTIC_DETENT, IC_DETENT, "卡位", "每圈 12 格  ·  清脆咔哒", 12, false},
    {HAPTIC_SWITCH, IC_SWITCH, "开关", "双稳态  ·  拨动切换", 2, true},
    {HAPTIC_DIMMER, IC_DIMMER, "调光", "0–100%  ·  细腻无级", 11, true},
    {HAPTIC_CURTAIN, IC_CURTAIN, "窗帘", "20 段行程  ·  软端点", 21, true},
};
#define MODE_N 5
#define CAR_STEP 92 /* 轮播间距 px */

typedef enum { PAGE_MENU = 0, PAGE_MODE } page_t;

/* 单生产者（按键任务）/ 单消费者（LVGL 任务）环形队列 */
static volatile uint8_t s_q[8];
static volatile uint32_t s_qw, s_qr;

static page_t s_page = PAGE_MENU;
static int s_sel = 0;
static int32_t s_saved_pos[MODE_N] = {0, 0, 0, 0, 0};
static int32_t s_last_pos = INT32_MIN;
static int s_last_sel = -1;
static float s_car_p = 0.0f;   /* 轮播显示位置（缓动） */
static float s_arc_v = 0.0f;   /* 圆环显示值 0–1000（缓动） */
static bool s_booted = false;

/* 控件 */
static lv_obj_t *s_st_icon, *s_st_txt, *s_st_cur, *s_st_tmp;
static lv_obj_t *s_menu, *s_car_disc[MODE_N], *s_car_icon[MODE_N], *s_m_name, *s_m_desc;
static lv_obj_t *s_mode, *s_arc, *s_meter, *s_stop_l, *s_stop_r, *s_val, *s_unit, *s_big;
static lv_obj_t *s_c_icon, *s_c_name, *s_c_sub, *s_c_sub2;
static lv_meter_scale_t *s_scale;
static lv_meter_indicator_t *s_ticks_ind;
static lv_obj_t *s_toast;

/* ================= 工具 ================= */

static lv_obj_t *mk_label(lv_obj_t *p, const lv_font_t *f, lv_color_t c, const char *t)
{
    lv_obj_t *l = lv_label_create(p);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    lv_label_set_text(l, t);
    return l;
}

static lv_obj_t *mk_layer(lv_obj_t *p)
{
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, SCR_W, SCR_H);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static void a_opa(void *o, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)o, (lv_opa_t)v, 0);
}

static void a_ty(void *o, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)o, (lv_coord_t)v, 0);
}

static void anim(lv_obj_t *o, lv_anim_exec_xcb_t cb, int32_t a, int32_t b, uint32_t t, uint32_t delay)
{
    lv_anim_t an;
    lv_anim_init(&an);
    lv_anim_set_var(&an, o);
    lv_anim_set_exec_cb(&an, cb);
    lv_anim_set_values(&an, a, b);
    lv_anim_set_time(&an, t);
    lv_anim_set_delay(&an, delay);
    lv_anim_set_path_cb(&an, lv_anim_path_ease_out);
    lv_anim_start(&an);
}

/** 入场：淡入 + 轻微上浮 */
static void reveal(lv_obj_t *o, uint32_t delay)
{
    lv_obj_set_style_opa(o, LV_OPA_TRANSP, 0);
    anim(o, a_opa, LV_OPA_TRANSP, LV_OPA_COVER, 260, delay);
    anim(o, a_ty, 8, 0, 320, delay);
}

static void toast(const char *txt)
{
    lv_label_set_text(s_toast, txt);
    lv_obj_align(s_toast, LV_ALIGN_CENTER, 0, 8);
    lv_obj_clear_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_toast);
    lv_anim_set_exec_cb(&a, a_opa);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_delay(&a, 900);
    lv_anim_set_time(&a, 380);
    lv_anim_start(&a);
    lv_obj_set_style_opa(s_toast, LV_OPA_COVER, 0);
}

/* ================= 构建 ================= */

static void build_status(lv_obj_t *scr)
{
    s_st_icon = mk_label(scr, &font_icon_16, C_TXT2, IC_ACT);
    lv_obj_set_pos(s_st_icon, 12, 5);
    s_st_txt = mk_label(scr, &font_cn_16, C_TXT2, "启动中");
    lv_obj_set_pos(s_st_txt, 32, 3);

    s_st_tmp = mk_label(scr, &font_cn_16, C_TXT2, "");
    lv_obj_align(s_st_tmp, LV_ALIGN_TOP_RIGHT, -12, 3);
    s_st_cur = mk_label(scr, &font_cn_16, C_TXT2, "");
    lv_obj_align_to(s_st_cur, s_st_tmp, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    lv_obj_t *hair = lv_img_create(scr);
    lv_img_set_src(hair, &img_hair);
    lv_obj_set_pos(hair, 10, 25);
}

static void build_menu(lv_obj_t *scr)
{
    s_menu = mk_layer(scr);
    for (int i = 0; i < MODE_N; i++) {
        lv_obj_t *d = lv_img_create(s_menu);
        lv_img_set_src(d, &img_disc_off);
        lv_img_set_pivot(d, 56, 56);
        lv_img_set_antialias(d, true);
        s_car_disc[i] = d;
        lv_obj_t *ic = mk_label(s_menu, &font_icon_40, C_TXT2, k_modes[i].icon);
        s_car_icon[i] = ic;
    }
    s_m_name = mk_label(s_menu, &font_cn_28, C_TXT1, "");
    s_m_desc = mk_label(s_menu, &font_cn_16, C_TXT2, "");
}

static void build_mode(lv_obj_t *scr)
{
    s_mode = mk_layer(scr);

    /* 刻度（lv_meter 只画刻度，不画背景/指针） */
    s_meter = lv_meter_create(s_mode);
    lv_obj_remove_style_all(s_meter);
    lv_obj_set_size(s_meter, 140, 140);
    lv_obj_set_pos(s_meter, 8, 28);
    lv_obj_set_style_pad_all(s_meter, 15, 0);
    s_scale = lv_meter_add_scale(s_meter);
    lv_meter_set_scale_range(s_meter, s_scale, 0, 100, 270, 135);
    lv_meter_set_scale_ticks(s_meter, s_scale, 12, 2, 5, C_TXT3);
    (void)s_ticks_ind;

    /* 渐变圆环 */
    s_arc = lv_arc_create(s_mode);
    lv_obj_set_size(s_arc, 140, 140);
    lv_obj_set_pos(s_arc, 8, 28);
    lv_arc_set_rotation(s_arc, 135);
    lv_arc_set_bg_angles(s_arc, 0, 270);
    lv_arc_set_range(s_arc, 0, 1000);
    lv_obj_remove_style(s_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_arc, 7, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc, C_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(s_arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 7, LV_PART_INDICATOR);
    lv_obj_set_style_arc_img_src(s_arc, &img_grad_ring, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_arc, true, LV_PART_INDICATOR);

    /* 端点标记（有界模式） */
    for (int k = 0; k < 2; k++) {
        lv_obj_t *d = lv_obj_create(s_mode);
        lv_obj_remove_style_all(d);
        lv_obj_set_size(d, 9, 9);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(0x0B0A09), 0);
        lv_obj_set_style_border_width(d, 2, 0);
        lv_obj_set_style_border_color(d, k ? C_TEAL : C_GOLD, 0);
        /* 圆环中心 (78,98)，半径 ≈ 66.5，端点角 135° / 45° */
        const float a = (k ? 45.0f : 135.0f) * 3.14159265f / 180.0f;
        lv_obj_set_pos(d, (lv_coord_t)(78 + 66.5f * cosf(a) - 4.5f), (lv_coord_t)(98 + 66.5f * sinf(a) - 4.5f));
        if (k) {
            s_stop_r = d;
        } else {
            s_stop_l = d;
        }
    }

    s_val = mk_label(s_mode, &font_num_52, C_TXT1, "0");
    s_big = mk_label(s_mode, &font_cn_28, C_TXT1, "");
    s_unit = mk_label(s_mode, &font_cn_16, C_TXT2, "");

    /* 右侧玻璃卡片 */
    lv_obj_t *card = lv_img_create(s_mode);
    lv_img_set_src(card, &img_card);
    lv_obj_set_pos(card, 156, 36);
    s_c_icon = mk_label(s_mode, &font_icon_16, C_GOLD, "");
    lv_obj_set_pos(s_c_icon, 170, 50);
    s_c_name = mk_label(s_mode, &font_cn_28, C_GOLD, "");
    lv_obj_set_pos(s_c_name, 168, 62);
    s_c_sub = mk_label(s_mode, &font_cn_20, C_TXT1, "");
    lv_obj_set_pos(s_c_sub, 170, 100);
    s_c_sub2 = mk_label(s_mode, &font_cn_16, C_TXT2, "");
    lv_obj_set_pos(s_c_sub2, 170, 124);
}

/* ================= 状态栏 ================= */

static void refresh_status(void)
{
    foc_power_t p = {};
    foc_get_power(&p);
    const bool ok = foc_is_ok();
    const foc_ctrl_mode_t c = foc_get_ctrl_mode();
    const char *ic, *txt;
    lv_color_t col;
    if (!foc_is_ready()) {
        ic = IC_ACT, txt = "电机校准中", col = C_GOLD;
    } else if (!ok) {
        ic = IC_ALERT, txt = "驱动未就绪 · 无力反馈", col = C_RED;
    } else if (c == FOC_CTRL_SENSOR) {
        ic = IC_STOP, txt = "已停止", col = C_TXT2;
    } else if (p.sleeping) {
        ic = IC_MOON, txt = "休眠", col = C_TEAL;
    } else {
        ic = IC_ACT, txt = "运行", col = C_GOLD;
    }
    lv_label_set_text(s_st_icon, ic);
    lv_obj_set_style_text_color(s_st_icon, col, 0);
    lv_label_set_text(s_st_txt, txt);
    lv_obj_set_style_text_color(s_st_txt, ok ? C_TXT2 : col, 0);

    /* LVGL 的 printf 不支持浮点，统一用定点整数格式化 */
    int dt10 = (int)lroundf(p.p_copper * RTH_K_PER_W * 10.0f);
    lv_label_set_text_fmt(s_st_tmp, IC_TEMP " +%d.%d°", dt10 / 10, dt10 % 10);
    lv_obj_align(s_st_tmp, LV_ALIGN_TOP_RIGHT, -12, 3);
    int ma = (int)lroundf(fabsf(p.i_est) * 1000.0f);
    lv_label_set_text_fmt(s_st_cur, IC_BOLT " %d.%02dA", ma / 1000, (ma % 1000) / 10);
    lv_obj_align_to(s_st_cur, s_st_tmp, LV_ALIGN_OUT_LEFT_MID, -12, 0);
}

/* ================= 菜单 ================= */

static void layout_carousel(float p)
{
    const int near = (int)lroundf(p);
    for (int i = 0; i < MODE_N; i++) {
        const float d = (float)i - p;
        const float ad = fabsf(d);
        const int cx = SCR_W / 2 + (int)lroundf(d * CAR_STEP);
        const int cy = 76;
        const int zoom = (int)(256 - fminf(ad, 1.6f) * 62);
        lv_img_set_zoom(s_car_disc[i], (uint16_t)zoom);
        lv_obj_set_pos(s_car_disc[i], cx - 56, cy - 56);
        lv_img_set_src(s_car_disc[i], i == near ? &img_disc_on : &img_disc_off);
        const lv_opa_t opa = (lv_opa_t)(ad > 2.2f ? 0 : (int)(255 * (1.0f - fminf(ad, 2.2f) / 2.4f)));
        lv_obj_set_style_opa(s_car_disc[i], opa, 0);
        lv_obj_set_style_text_color(s_car_icon[i], i == near ? C_GOLD_HI : C_TXT2, 0);
        lv_obj_set_style_text_opa(s_car_icon[i], (lv_opa_t)(i == near ? 255 : opa * 0.7f), 0);
        lv_obj_update_layout(s_car_icon[i]);
        lv_obj_set_pos(s_car_icon[i], cx - lv_obj_get_width(s_car_icon[i]) / 2,
                       cy - lv_obj_get_height(s_car_icon[i]) / 2);
    }
}

static void set_menu_texts(int sel)
{
    lv_label_set_text(s_m_name, k_modes[sel].name);
    lv_obj_align(s_m_name, LV_ALIGN_TOP_MID, 0, 120);
    lv_label_set_text(s_m_desc, k_modes[sel].desc);
    lv_obj_align(s_m_desc, LV_ALIGN_TOP_MID, 0, 150);
}

static void refresh_menu(const foc_haptic_snapshot_t *hs)
{
    float target = (float)s_sel;
    if (hs->mode == HAPTIC_MENU && hs->width_rad > 0.0f) {
        s_sel = (int)hs->position;
        const float frac = (hs->shaft_rad - (float)hs->position * hs->width_rad) / hs->width_rad;
        /* 格内跟手 60%，过 snap 点后整体吸附到下一格 */
        target = (float)hs->position + fmaxf(-0.5f, fminf(0.5f, frac)) * 0.6f;
    }
    s_car_p += (target - s_car_p) * 0.35f;
    if (fabsf(target - s_car_p) < 0.002f) {
        s_car_p = target;
    }
    layout_carousel(s_car_p);
    if (s_sel != s_last_sel) {
        s_last_sel = s_sel;
        set_menu_texts(s_sel);
        anim(s_m_name, a_opa, 90, 255, 220, 0);
        printf("[ui] menu sel=%d %s\n", s_sel, k_modes[s_sel].name);
    }
}

/* ================= 详情 ================= */

static void enter_menu(void)
{
    s_page = PAGE_MENU;
    if (foc_is_ok() && foc_get_ctrl_mode() != FOC_CTRL_HAPTIC) {
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    }
    foc_set_haptic_mode_pos(HAPTIC_MENU, s_sel);
    lv_obj_clear_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    s_last_sel = -1;
    s_car_p = (float)s_sel;
    reveal(s_menu, 0);
    printf("[ui] page=menu sel=%d %s\n", s_sel, k_modes[s_sel].name);
}

static void enter_mode(void)
{
    s_page = PAGE_MODE;
    const mode_item_t *m = &k_modes[s_sel];
    if (foc_is_ok() && foc_get_ctrl_mode() != FOC_CTRL_HAPTIC) {
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    }
    foc_set_haptic_mode_pos(m->mode, s_saved_pos[s_sel]);

    lv_meter_set_scale_ticks(s_meter, s_scale, (uint16_t)m->ticks, 2, m->ticks > 24 ? 4 : 6, C_TXT3);
    if (m->bounded) {
        lv_obj_clear_flag(s_stop_l, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_stop_r, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_stop_l, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_stop_r, LV_OBJ_FLAG_HIDDEN);
    }
    lv_label_set_text(s_c_icon, m->icon);
    lv_label_set_text(s_c_name, m->name);
    lv_obj_add_flag(s_menu, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    s_last_pos = INT32_MIN;
    s_arc_v = 0.0f;
    reveal(s_mode, 0);
    printf("[ui] page=mode %s pos=%ld\n", m->name, (long)s_saved_pos[s_sel]);
}

static void center_value(void)
{
    lv_obj_align(s_val, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_update_layout(s_val);
    lv_obj_set_pos(s_val, 78 - lv_obj_get_width(s_val) / 2, 98 - lv_obj_get_height(s_val) / 2 - 6);
    lv_obj_update_layout(s_big);
    lv_obj_set_pos(s_big, 78 - lv_obj_get_width(s_big) / 2, 98 - lv_obj_get_height(s_big) / 2 - 6);
    lv_obj_update_layout(s_unit);
    lv_obj_set_pos(s_unit, 78 - lv_obj_get_width(s_unit) / 2, 122);
}

static void refresh_mode(const foc_haptic_snapshot_t *hs)
{
    const mode_item_t *m = &k_modes[s_sel];
    if (hs->mode != m->mode) {
        return;
    }
    const float deg = hs->shaft_rad * 57.29578f;
    float target = 0.0f;
    const bool changed = (hs->position != s_last_pos);
    bool num = true;

    switch (m->mode) {
    case HAPTIC_FREE: {
        float a = fmodf(deg, 360.0f);
        if (a < 0) {
            a += 360.0f;
        }
        target = a / 360.0f * 1000.0f;
        lv_label_set_text_fmt(s_val, "%d", (int)a);
        lv_label_set_text(s_unit, "角度 °");
        int t100 = (int)lroundf(deg / 3.6f);
        int a100 = t100 < 0 ? -t100 : t100;
        lv_label_set_text_fmt(s_c_sub, "%c%d.%02d 圈", t100 < 0 ? '-' : '+', a100 / 100, a100 % 100);
        lv_label_set_text(s_c_sub2, "零阻尼 · 自由旋转");
        break;
    }
    case HAPTIC_DETENT: {
        int32_t idx = hs->position % 12;
        if (idx < 0) {
            idx += 12;
        }
        target = (float)idx * 1000.0f / 11.0f;
        lv_label_set_text_fmt(s_val, "%ld", (long)idx);
        lv_label_set_text(s_unit, "档位");
        lv_label_set_text_fmt(s_c_sub, "第 %ld / 12 格", (long)idx + 1);
        lv_label_set_text_fmt(s_c_sub2, "累计 %+ld", (long)hs->position);
        break;
    }
    case HAPTIC_SWITCH: {
        const bool on = hs->position > 0;
        target = on ? 1000.0f : 0.0f;
        num = false;
        lv_label_set_text(s_big, on ? "开" : "关");
        lv_obj_set_style_text_color(s_big, on ? C_GOLD_HI : C_TXT2, 0);
        lv_label_set_text(s_unit, "灯光");
        lv_label_set_text(s_c_sub, on ? "已打开" : "已关闭");
        lv_label_set_text(s_c_sub2, "拨过中点即切换");
        break;
    }
    case HAPTIC_DIMMER:
    case HAPTIC_CURTAIN: {
        const int pct = (int)lroundf(hs->value_norm * 100.0f);
        target = (float)pct * 10.0f;
        lv_label_set_text_fmt(s_val, "%d", pct);
        lv_label_set_text(s_unit, m->mode == HAPTIC_DIMMER ? "亮度 %" : "开合 %");
        if (m->mode == HAPTIC_DIMMER) {
            lv_label_set_text(s_c_sub, pct == 0 ? "已关灯" : (pct > 80 ? "明亮" : (pct > 30 ? "柔和" : "微光")));
            lv_label_set_text(s_c_sub2, "两端有软限位");
        } else {
            lv_label_set_text(s_c_sub, pct == 0 ? "全关" : (pct == 100 ? "全开" : "半开"));
            lv_label_set_text_fmt(s_c_sub2, "行程 %ld / 20", (long)hs->position);
        }
        break;
    }
    default:
        break;
    }
    if (num) {
        lv_obj_clear_flag(s_val, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_big, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_val, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_big, LV_OBJ_FLAG_HIDDEN);
    }
    s_arc_v += (target - s_arc_v) * 0.4f;
    if (fabsf(target - s_arc_v) < 1.0f) {
        s_arc_v = target;
    }
    lv_arc_set_value(s_arc, (int16_t)s_arc_v);
    center_value();

    if (changed) {
        s_last_pos = hs->position;
        s_saved_pos[s_sel] = hs->position;
        printf("[ui] %s pos=%ld value=%.0f%% angle=%.1f°\n", m->name, (long)hs->position,
               (double)(hs->value_norm * 100.0f), (double)deg);
    }
}

/* ================= 事件 / 定时器 ================= */

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
    (void)t;
    while (s_qr != s_qw) {
        handle_event((ui_evt_t)s_q[s_qr % 8]);
        s_qr++;
    }
    if (!s_booted) {
        refresh_status();
        if (!foc_is_ready()) {
            return;
        }
        s_booted = true;
        enter_menu();
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

void ui_knob_post(ui_evt_t evt)
{
    if (s_qw - s_qr < 8) {
        s_q[s_qw % 8] = (uint8_t)evt;
        s_qw++;
    }
}

void ui_knob_create(lv_obj_t *scr)
{
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *bg = lv_img_create(scr);
    lv_img_set_src(bg, &img_bg);
    lv_obj_set_pos(bg, 0, 0);

    build_status(scr);
    build_menu(scr);
    build_mode(scr);
    lv_obj_add_flag(s_mode, LV_OBJ_FLAG_HIDDEN);
    layout_carousel(0.0f);
    set_menu_texts(0);

    s_toast = mk_label(scr, &font_cn_20, C_TXT1, "");
    lv_obj_set_style_bg_color(s_toast, lv_color_hex(0x1E1C1A), 0);
    lv_obj_set_style_bg_opa(s_toast, LV_OPA_90, 0);
    lv_obj_set_style_border_color(s_toast, C_GOLD, 0);
    lv_obj_set_style_border_opa(s_toast, LV_OPA_40, 0);
    lv_obj_set_style_border_width(s_toast, 1, 0);
    lv_obj_set_style_radius(s_toast, 14, 0);
    lv_obj_set_style_pad_hor(s_toast, 16, 0);
    lv_obj_set_style_pad_ver(s_toast, 6, 0);
    lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);

    lv_timer_create(ui_timer_cb, 30, NULL);
}

#ifndef UI_SIM
#include "esp_log.h"
#include "lcd_tdisplay.h"

void ui_knob_start(void)
{
    if (!lcd_tdisplay_start(0, 2)) {
        ESP_LOGE("ui", "display init failed — UI disabled");
        return;
    }
    lvgl_lock(-1);
    ui_knob_create(lv_scr_act());
    lvgl_unlock();
    lcd_set_backlight(85);
    ESP_LOGI("ui", "UI started");
}
#endif
