/**
 * @file app_main.cpp
 * @brief 应用入口：NVS → 启动 FOC 任务 → 启动串口控制台。
 *
 * Phase 1 只做电机闭环与触感 bring-up；MQTT / UI 后续阶段再加。
 */

#include "esp_log.h"
#include "nvs_flash.h"

#include "button_ctrl.h"
#include "console_cmd.h"
#include "foc_task.h"
#include "ui/ui_knob.h"

static const char *TAG = "app";

extern "C" void app_main(void)
{
    /* NVS：后续存触感参数 / Wi-Fi 等会用到，先初始化 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Smart Knob Panel — 旋钮屏 UI 版");

    foc_task_start();
    /* 屏幕 + LVGL：core 0 低优先级；FOC 独占 core 1 */
    ui_knob_start();
    /* 主交互：GPIO14 短按切模式 / 长按急停；日志走串口 */
    button_ctrl_start();

    /* 控制台仍保留，但监视器若超时打字失败也不影响按键 */
    console_cmd_start();
}
