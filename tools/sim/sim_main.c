/**
 * PC 预览：用与固件相同的 main/ui/ui_knob.c 渲染关键画面到 PNG（无需硬件）。
 * 编译运行：tools/sim/run.sh  → 输出 tools/sim/out/*.png
 */
#include <stdio.h>
#include <string.h>
#include "lvgl.h"
#include "../../main/foc_task.h"
#include "../../main/ui/ui_knob.h"

#define W 320
#define H 170
static lv_color_t s_fb[W * H];
static lv_color_t s_buf[W * H];

/* ---------- foc_task 桩 ---------- */
static foc_haptic_snapshot_t s_hs = {HAPTIC_MENU, 0, 5, 0, 0, 0.5236f};
static bool s_ok = true, s_ready = true, s_sleep = false;
static foc_ctrl_mode_t s_ctrl = FOC_CTRL_HAPTIC;
static float s_i = 0.0f, s_irms = 0.0f;

static void mode_params(haptic_mode_t m, int32_t *n, float *w)
{
    switch (m) {
    case HAPTIC_MENU: *n = 5; *w = 0.5236f; break;
    case HAPTIC_DETENT: *n = 0; *w = 0.5236f; break;
    case HAPTIC_SWITCH: *n = 2; *w = 0.698f; break;
    case HAPTIC_DIMMER: *n = 101; *w = 0.0471f; break;
    case HAPTIC_CURTAIN: *n = 21; *w = 0.314f; break;
    default: *n = 0; *w = 0.1745f; break;
    }
}
void foc_set_haptic_mode_pos(haptic_mode_t mode, int32_t pos)
{
    s_hs.mode = mode;
    mode_params(mode, &s_hs.num_positions, &s_hs.width_rad);
    s_hs.position = pos;
    s_hs.shaft_rad = pos * s_hs.width_rad;
    s_hs.value_norm = s_hs.num_positions > 1 ? (float)pos / (s_hs.num_positions - 1) : 0;
}
void foc_get_haptic_snapshot(foc_haptic_snapshot_t *o) { *o = s_hs; }
bool foc_is_ok(void) { return s_ok; }
bool foc_is_ready(void) { return s_ready; }
foc_ctrl_mode_t foc_get_ctrl_mode(void) { return s_ctrl; }
void foc_set_ctrl_mode(foc_ctrl_mode_t m) { s_ctrl = m; }
void foc_stop(void) { s_ctrl = FOC_CTRL_SENSOR; }
void foc_get_power(foc_power_t *p)
{
    memset(p, 0, sizeof(*p));
    p->i_est = s_i; p->i_rms = s_irms; p->p_copper = 1.5f * s_irms * s_irms * 2.5f; p->sleeping = s_sleep;
}

/* ---------- 显示 ---------- */
static void flush(lv_disp_drv_t *d, const lv_area_t *a, lv_color_t *c)
{
    for (int y = a->y1; y <= a->y2; y++) {
        memcpy(&s_fb[y * W + a->x1], c, (a->x2 - a->x1 + 1) * sizeof(lv_color_t));
        c += a->x2 - a->x1 + 1;
    }
    lv_disp_flush_ready(d);
}

static void run(int ms)
{
    for (int t = 0; t < ms; t += 5) {
        lv_tick_inc(5);
        lv_timer_handler();
    }
}

static void shot(const char *name)
{
    lv_obj_invalidate(lv_scr_act());
    run(30);
    char p[256];
    snprintf(p, sizeof p, "out/%s.raw", name);
    FILE *f = fopen(p, "wb");
    fwrite(s_fb, sizeof(lv_color_t), W * H, f);
    fclose(f);
    printf("shot %s\n", name);
}

/* 模拟拧到某格（含格内偏移） */
static void turn(int32_t pos, float frac)
{
    s_hs.position = pos;
    s_hs.shaft_rad = (pos + frac) * s_hs.width_rad;
    s_hs.value_norm = s_hs.num_positions > 1 ? (float)pos / (s_hs.num_positions - 1) : 0;
}

int main(void)
{
    lv_init();
    static lv_disp_draw_buf_t db;
    lv_disp_draw_buf_init(&db, s_buf, NULL, W * H);
    static lv_disp_drv_t dd;
    lv_disp_drv_init(&dd);
    dd.hor_res = W; dd.ver_res = H; dd.flush_cb = flush; dd.draw_buf = &db;
    lv_disp_drv_register(&dd);

    ui_knob_create(lv_scr_act());
    s_i = 0.08f; s_irms = 0.06f;
    run(1500);
    shot("01_menu_free");
    turn(0, 0.45f); run(400); shot("02_menu_turning");
    turn(3, 0.0f); run(1200); shot("03_menu_dimmer");

    ui_knob_post(UI_EVT_SHORT); run(100);       /* 进入调光 */
    turn(62, 0); run(1500); shot("04_dimmer_62");
    ui_knob_post(UI_EVT_LONG); run(200);        /* 返回菜单 */
    turn(1, 0); run(1000); ui_knob_post(UI_EVT_SHORT); run(100);
    turn(7, 0); run(1500); shot("05_detent_7");
    ui_knob_post(UI_EVT_LONG); run(200);
    turn(2, 0); run(1000); ui_knob_post(UI_EVT_SHORT); run(100);
    turn(1, 0); run(1500); shot("06_switch_on");
    ui_knob_post(UI_EVT_LONG); run(200);
    turn(4, 0); run(1000); ui_knob_post(UI_EVT_SHORT); run(100);
    turn(9, 0); run(1500); shot("07_curtain_45");
    ui_knob_post(UI_EVT_LONG); run(200);
    turn(0, 0); run(1000); ui_knob_post(UI_EVT_SHORT); run(100);
    s_hs.shaft_rad = 2.3f; run(1500); shot("08_free_angle");

    /* 状态：休眠 / 驱动未就绪 */
    s_sleep = true; s_i = 0; s_irms = 0.01f; run(400); shot("09_status_sleep");
    ui_knob_post(UI_EVT_LONG); run(200);
    s_ok = false; s_ctrl = FOC_CTRL_SENSOR; s_sleep = false; s_i = 0; s_irms = 0;
    turn(3, 0); run(1500); shot("10_degraded_menu");
    return 0;
}
