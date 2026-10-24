/**
 * @file lcd_tdisplay.c
 * @brief T-Display-S3 ST7789(i80) + LVGL 8.3 移植。
 *
 * 线程模型：LVGL 只在 lvgl 任务里跑（核 0、低优先级），FOC 任务独占核 1 不受刷屏影响。
 * 刷屏走 i80 DMA，双缓冲 320×40 行（内部 RAM，2×25.6KB），传输完成中断里通知 LVGL。
 */

#include "lcd_tdisplay.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "lcd";

#define PIN_LCD_BL   38
#define PIN_LCD_PWR  15
#define PIN_LCD_RST  5
#define PIN_LCD_CS   6
#define PIN_LCD_DC   7
#define PIN_LCD_WR   8
#define PIN_LCD_RD   9
static const int k_data_pins[8] = {39, 40, 41, 42, 45, 46, 47, 48};

#define LCD_PCLK_HZ      (10 * 1000 * 1000) /* 官方 16MHz 偶有花屏，取 10MHz 稳定 */
#define LCD_BUF_LINES    40
#define LCD_X_GAP        0
#define LCD_Y_GAP        35                 /* 170 行屏在 ST7789 240 行 GRAM 中的偏移 */
#define BL_LEDC_CH       LEDC_CHANNEL_4     /* 0–2 被电机 3PWM 占用 */
#define BL_LEDC_TIMER    LEDC_TIMER_1       /* TIMER_0 = 电机 20kHz */

static SemaphoreHandle_t s_lvgl_mux;
static esp_lcd_panel_handle_t s_panel;
static lv_disp_drv_t s_disp_drv;

static bool on_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ev, void *ctx)
{
    lv_disp_flush_ready((lv_disp_drv_t *)ctx);
    return false;
}

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *a, lv_color_t *map)
{
    esp_lcd_panel_draw_bitmap(s_panel, a->x1, a->y1, a->x2 + 1, a->y2 + 1, map);
}

static void tick_cb(void *arg)
{
    lv_tick_inc(2);
}

bool lvgl_lock(int timeout_ms)
{
    const TickType_t t = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_lvgl_mux, t) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(s_lvgl_mux);
}

void lcd_set_backlight(int percent)
{
    percent = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH, (uint32_t)(percent * 1023 / 100));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CH);
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task running (core %d)", xPortGetCoreID());
    while (true) {
        uint32_t wait = 10;
        if (lvgl_lock(-1)) {
            wait = lv_timer_handler();
            lvgl_unlock();
        }
        if (wait < 5) {
            wait = 5;
        } else if (wait > 30) {
            wait = 30;
        }
        vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

bool lcd_tdisplay_start(int core, int priority)
{
    /* 供电 + RD 常高 */
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LCD_PWR) | (1ULL << PIN_LCD_RD);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)PIN_LCD_PWR, 1);
    gpio_set_level((gpio_num_t)PIN_LCD_RD, 1);

    /* 背光 PWM（先关，避免上电花屏） */
    ledc_timer_config_t bt = {};
    bt.speed_mode = LEDC_LOW_SPEED_MODE;
    bt.duty_resolution = LEDC_TIMER_10_BIT;
    bt.timer_num = BL_LEDC_TIMER;
    bt.freq_hz = 5000;
    bt.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&bt));
    ledc_channel_config_t bc = {};
    bc.gpio_num = PIN_LCD_BL;
    bc.speed_mode = LEDC_LOW_SPEED_MODE;
    bc.channel = BL_LEDC_CH;
    bc.timer_sel = BL_LEDC_TIMER;
    bc.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&bc));

    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_cfg = {};
    bus_cfg.dc_gpio_num = PIN_LCD_DC;
    bus_cfg.wr_gpio_num = PIN_LCD_WR;
    bus_cfg.clk_src = LCD_CLK_SRC_DEFAULT;
    for (int i = 0; i < 8; i++) {
        bus_cfg.data_gpio_nums[i] = k_data_pins[i];
    }
    bus_cfg.bus_width = 8;
    bus_cfg.max_transfer_bytes = LCD_H_RES * LCD_BUF_LINES * sizeof(uint16_t);
    esp_err_t err = esp_lcd_new_i80_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i80 bus: %s", esp_err_to_name(err));
        return false;
    }

    esp_lcd_panel_io_handle_t pio = NULL;
    esp_lcd_panel_io_i80_config_t io_cfg = {};
    io_cfg.cs_gpio_num = PIN_LCD_CS;
    io_cfg.pclk_hz = LCD_PCLK_HZ;
    io_cfg.trans_queue_depth = 10;
    io_cfg.on_color_trans_done = on_trans_done;
    io_cfg.user_ctx = &s_disp_drv;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    io_cfg.dc_levels.dc_idle_level = 0;
    io_cfg.dc_levels.dc_cmd_level = 0;
    io_cfg.dc_levels.dc_dummy_level = 0;
    io_cfg.dc_levels.dc_data_level = 1;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &io_cfg, &pio));

    esp_lcd_panel_dev_config_t pcfg = {};
    pcfg.reset_gpio_num = PIN_LCD_RST;
    pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    pcfg.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(pio, &pcfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    /* 横屏：USB 口在右侧 */
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_X_GAP, LCD_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    lv_init();
    const size_t bytes = LCD_H_RES * LCD_BUF_LINES * sizeof(lv_color_t);
    lv_color_t *b1 = (lv_color_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t *b2 = (lv_color_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!b1 || !b2) {
        ESP_LOGE(TAG, "draw buffer alloc failed");
        return false;
    }
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, b1, b2, LCD_H_RES * LCD_BUF_LINES);
    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res = LCD_H_RES;
    s_disp_drv.ver_res = LCD_V_RES;
    s_disp_drv.flush_cb = flush_cb;
    s_disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&s_disp_drv);

    const esp_timer_create_args_t targs = {.callback = tick_cb, .name = "lv_tick"};
    esp_timer_handle_t th;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &th));
    ESP_ERROR_CHECK(esp_timer_start_periodic(th, 2000));

    s_lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 6144, NULL, priority, NULL, core);
    ESP_LOGI(TAG, "ST7789 170x320 i80 init OK (pclk %d MHz)", LCD_PCLK_HZ / 1000000);
    return true;
}
