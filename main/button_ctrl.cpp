/**
 * @file button_ctrl.cpp
 * @brief GPIO14 用户键：短按切测试档，长按急停。状态打串口日志。
 */

#include "button_ctrl.h"

#include <cstdio>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "foc_task.h"

static const char *TAG = "btn";

#ifndef BOARD_USER_BTN_GPIO
#define BOARD_USER_BTN_GPIO GPIO_NUM_14
#endif

static void button_task(void *arg)
{
    (void)arg;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BOARD_USER_BTN_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    printf("\n*** 按键 GPIO%d：短按=停→自由→卡位→停（后两档可一直跑）\n"
           "*** 长按=急停。自由/卡位静止零电流，空闲自动休眠\n\n",
           (int)BOARD_USER_BTN_GPIO);
    ESP_LOGI(TAG, "user button on GPIO%d (active-low)", (int)BOARD_USER_BTN_GPIO);

    while (!foc_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("*** FOC 就绪，可以按键测试电机\n\n");

    bool prev_down = false;
    TickType_t down_tick = 0;
    const TickType_t long_ms = pdMS_TO_TICKS(800);
    const TickType_t debounce = pdMS_TO_TICKS(30);

    while (true) {
        /* 按下=低电平（对地） */
        const bool down = (gpio_get_level(BOARD_USER_BTN_GPIO) == 0);

        if (down && !prev_down) {
            down_tick = xTaskGetTickCount();
            prev_down = true;
        } else if (!down && prev_down) {
            const TickType_t held = xTaskGetTickCount() - down_tick;
            vTaskDelay(debounce);
            if (gpio_get_level(BOARD_USER_BTN_GPIO) != 0) {
                if (held >= long_ms) {
                    printf("\n>>> 长按 → 急停\n");
                    foc_stop();
                } else {
                    printf("\n>>> 短按 → 下一档\n");
                    foc_demo_next();
                }
                prev_down = false;
            }
        } else if (down && prev_down) {
            /* 按住超过长按阈值就立刻急停，不必等松手 */
            if ((xTaskGetTickCount() - down_tick) >= long_ms) {
                printf("\n>>> 长按 → 急停\n");
                foc_stop();
                /* 等松手，避免连触发 */
                while (gpio_get_level(BOARD_USER_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                prev_down = false;
                vTaskDelay(debounce);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/** 每秒打一行状态，方便不打字也能看 */
static void status_log_task(void *arg)
{
    (void)arg;
    while (!foc_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    while (true) {
        float shaft = 0, sens = 0, vel = 0;
        foc_get_position(&shaft, &sens, &vel);
        foc_power_t pw = {};
        foc_get_power(&pw);
        const float deg = sens * 180.0f / 3.14159265f;
        printf("[stat] ctrl=%-8s ang=%6.1f° shaft=%+.2f vel=%+.2f I=%+.3fA Irms=%.3fA Pcu=%.0fmW sleep=%d EN=%d\n",
               foc_ctrl_mode_name(foc_get_ctrl_mode()),
               deg, shaft, vel, pw.i_est, pw.i_rms, pw.p_copper * 1000.0f,
               (int)pw.sleeping, gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void button_ctrl_start(void)
{
    xTaskCreatePinnedToCore(button_task, "btn", 3072, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(status_log_task, "stat", 3072, nullptr, 2, nullptr, 0);
}
