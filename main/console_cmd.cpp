/**
 * @file console_cmd.cpp
 * @brief 串口命令。烧录后监视器出现 knob> 即可交互。
 *
 * 电机 bring-up：
 *   stream off
 *   test spin     — 开环慢转几秒
 *   test hold     — 闭环锁当前角
 *   test detent   — 虚拟卡位
 *   stop          — 急停回 sensor
 */

#include "console_cmd.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "foc_task.h"
#include "haptic/haptic_mode.h"

static const char *TAG = "console";

static int cmd_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("\nSmart Knob Panel 命令:\n"
           "  help / status / stream <on|off>\n"
           "  stop                         急停（回 sensor，出力=0）\n"
           "  vlim <V>                     限压（默认 6，供电可到 12）\n"
           "  ctrl <sensor|openloop|angle|haptic>\n"
           "  target <float>               开环速度 rad/s 或 闭环角 rad\n"
           "  mode <free|detent|switch|dimmer|curtain>\n"
           "  detent <2..64>\n"
           "\n快捷测试（会自动 stream off）:\n"
           "  test spin [秒] [rad/s]       开环慢转，默认 4s @ 2 rad/s\n"
           "  test hold [偏移rad]          闭环锁角，默认锁当前，可加小偏移\n"
           "  test detent [档数]           触感卡位，默认 12\n"
           "  test step <rad>              从当前角步进到 +rad（闭环）\n"
           "\n建议顺序: test spin -> stop -> test hold -> test detent -> stop\n"
           "注意: 电机需独立 7.4–12V，USB 不够力气；手勿挡轴！\n\n");
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    float shaft = 0, sens = 0, vel = 0, torque = 0;
    foc_get_position(&shaft, &sens, &vel);
    foc_get_telemetry(nullptr, nullptr, &torque);
    const float deg = sens * 180.0f / 3.14159265f;
    const float turns = shaft / (2.0f * 3.14159265f);
    printf("\nctrl=%s haptic=%s stream=%s target=%.3f Vlim=%.1f\n"
           "sensor=%.1f° (%.3f rad)  shaft=%+.2f rad (%+.2f turns)\n"
           "vel=%.3f rad/s  torque=%.3f\n",
           foc_ctrl_mode_name(foc_get_ctrl_mode()),
           haptic_mode_name(foc_get_haptic_mode()),
           foc_get_angle_stream() ? "on" : "off",
           foc_get_target(),
           foc_get_voltage_limit(),
           deg, sens, shaft, turns,
           vel, torque);
    return 0;
}

static int cmd_stream(int argc, char **argv)
{
    if (argc < 2) {
        printf("stream is %s — usage: stream <on|off>\n",
               foc_get_angle_stream() ? "on" : "off");
        return 1;
    }
    if (!strcmp(argv[1], "on") || !strcmp(argv[1], "1")) {
        foc_set_angle_stream(true);
    } else if (!strcmp(argv[1], "off") || !strcmp(argv[1], "0")) {
        foc_set_angle_stream(false);
        printf("\n");
    } else {
        printf("usage: stream <on|off>\n");
        return 1;
    }
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    foc_stop();
    printf("stopped → sensor\n");
    return 0;
}

static int cmd_vlim(int argc, char **argv)
{
    if (argc < 2) {
        printf("Vlim=%.1f V — usage: vlim <volts>\n", foc_get_voltage_limit());
        return 1;
    }
    float v = strtof(argv[1], nullptr);
    foc_set_voltage_limit(v);
    printf("Vlim=%.1f V\n", foc_get_voltage_limit());
    return 0;
}

static int cmd_ctrl(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: ctrl <sensor|openloop|angle|haptic>\n");
        return 1;
    }
    foc_set_angle_stream(false);
    if (!strcmp(argv[1], "sensor")) {
        foc_set_ctrl_mode(FOC_CTRL_SENSOR);
    } else if (!strcmp(argv[1], "openloop")) {
        foc_set_ctrl_mode(FOC_CTRL_OPENLOOP);
        printf("openloop target=%.2f rad/s  (改速: target <v> / 停: stop)\n", foc_get_target());
    } else if (!strcmp(argv[1], "angle")) {
        foc_set_ctrl_mode(FOC_CTRL_ANGLE);
        printf("angle hold target=%.3f rad  (改角: target <rad> / 停: stop)\n", foc_get_target());
    } else if (!strcmp(argv[1], "haptic")) {
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    } else {
        printf("unknown ctrl '%s'\n", argv[1]);
        return 1;
    }
    return 0;
}

static int cmd_mode(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: mode <free|detent|switch|dimmer|curtain>\n");
        return 1;
    }
    haptic_mode_t m = HAPTIC_FREE;
    if (!strcmp(argv[1], "free")) {
        m = HAPTIC_FREE;
    } else if (!strcmp(argv[1], "detent")) {
        m = HAPTIC_DETENT;
    } else if (!strcmp(argv[1], "switch")) {
        m = HAPTIC_SWITCH;
    } else if (!strcmp(argv[1], "dimmer")) {
        m = HAPTIC_DIMMER;
    } else if (!strcmp(argv[1], "curtain")) {
        m = HAPTIC_CURTAIN;
    } else {
        printf("unknown mode '%s'\n", argv[1]);
        return 1;
    }
    foc_set_angle_stream(false);
    foc_set_haptic_mode(m);
    foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    printf("haptic %s — 用手拧旋钮感受；stop 退出\n", haptic_mode_name(m));
    return 0;
}

static int cmd_target(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: target <float>\n");
        return 1;
    }
    float v = strtof(argv[1], nullptr);
    foc_set_target(v);
    printf("target=%.3f\n", v);
    return 0;
}

static int cmd_detent(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: detent <count>\n");
        return 1;
    }
    int n = atoi(argv[1]);
    if (n < 2 || n > 64) {
        printf("detent count must be 2..64\n");
        return 1;
    }
    foc_set_angle_stream(false);
    foc_set_detent_count(n);
    foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
    printf("detent %d — 拧旋钮应有卡位感；stop 退出\n", n);
    return 0;
}

struct spin_args_t {
    int seconds;
    float rad_s;
};

static void spin_auto_stop_task(void *arg)
{
    spin_args_t *a = (spin_args_t *)arg;
    const int sec = a->seconds;
    const float rad_s = a->rad_s;
    free(a);

    foc_set_angle_stream(false);
    foc_set_ctrl_mode(FOC_CTRL_OPENLOOP);
    foc_set_target(rad_s);
    printf("\n[test spin] openloop %.1f rad/s，%d 秒后自动 stop\n", rad_s, sec);
    printf("  若电机不转: 查 VM 供电/三相线；反转则对调任意两相\n");

    vTaskDelay(pdMS_TO_TICKS(sec * 1000));
    foc_stop();
    printf("[test spin] done → sensor\n");
    vTaskDelete(nullptr);
}

static int cmd_test(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: test <spin|hold|detent|step> ...\n");
        return 1;
    }

    if (!strcmp(argv[1], "spin")) {
        int sec = 4;
        float rad_s = 2.0f;
        if (argc >= 3) {
            sec = atoi(argv[2]);
        }
        if (argc >= 4) {
            rad_s = strtof(argv[3], nullptr);
        }
        if (sec < 1) {
            sec = 1;
        }
        if (sec > 30) {
            sec = 30;
        }
        if (rad_s < -10.f) {
            rad_s = -10.f;
        }
        if (rad_s > 10.f) {
            rad_s = 10.f;
        }

        spin_args_t *a = (spin_args_t *)malloc(sizeof(spin_args_t));
        if (!a) {
            return 1;
        }
        a->seconds = sec;
        a->rad_s = rad_s;
        xTaskCreate(spin_auto_stop_task, "test_spin", 3072, a, 3, nullptr);
        return 0;
    }

    if (!strcmp(argv[1], "hold")) {
        float offset = 0.0f;
        if (argc >= 3) {
            offset = strtof(argv[2], nullptr);
        }
        float shaft = 0, sens = 0, vel = 0;
        foc_get_position(&shaft, &sens, &vel);
        foc_set_angle_stream(false);
        foc_set_ctrl_mode(FOC_CTRL_ANGLE);
        foc_set_target(shaft + offset);
        printf("[test hold] 锁在 %.3f rad (当前%+.3f + 偏移%+.3f)\n",
               shaft + offset, shaft, offset);
        printf("  用手拧应有回弹；乱抖/发热 → stop，查极对数/相序\n");
        return 0;
    }

    if (!strcmp(argv[1], "step")) {
        if (argc < 3) {
            printf("usage: test step <delta_rad>   例: test step 0.5\n");
            return 1;
        }
        float delta = strtof(argv[2], nullptr);
        float shaft = 0, sens = 0, vel = 0;
        foc_get_position(&shaft, &sens, &vel);
        foc_set_angle_stream(false);
        foc_set_ctrl_mode(FOC_CTRL_ANGLE);
        foc_set_target(shaft + delta);
        printf("[test step] %.3f → %.3f rad\n", shaft, shaft + delta);
        return 0;
    }

    if (!strcmp(argv[1], "detent")) {
        int n = 12;
        if (argc >= 3) {
            n = atoi(argv[2]);
        }
        if (n < 2) {
            n = 2;
        }
        if (n > 64) {
            n = 64;
        }
        foc_set_angle_stream(false);
        foc_set_detent_count(n);
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
        printf("[test detent] %d 档 — 慢慢拧一圈应感到均匀卡位；stop 退出\n", n);
        return 0;
    }

    printf("unknown test '%s' — try: spin | hold | detent | step\n", argv[1]);
    return 1;
}

static void register_cmd(const char *command,
                         const char *help,
                         const char *hint,
                         esp_console_cmd_func_t func)
{
    esp_console_cmd_t cmd = {};
    cmd.command = command;
    cmd.help = help;
    cmd.hint = hint;
    cmd.func = func;
    cmd.argtable = nullptr;
    cmd.func_w_context = nullptr;
    cmd.context = nullptr;
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void console_cmd_start(void)
{
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "knob> ";
    repl_config.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    register_cmd("help", "显示帮助", nullptr, &cmd_help);
    register_cmd("status", "打印状态", nullptr, &cmd_status);
    register_cmd("stream", "角度流开关", "<on|off>", &cmd_stream);
    register_cmd("stop", "急停回 sensor", nullptr, &cmd_stop);
    register_cmd("vlim", "设置限压", "<volts>", &cmd_vlim);
    register_cmd("ctrl", "控制路径", "<sensor|openloop|angle|haptic>", &cmd_ctrl);
    register_cmd("mode", "触感模式", "<free|detent|switch|dimmer|curtain>", &cmd_mode);
    register_cmd("target", "开环速度/闭环角", "<float>", &cmd_target);
    register_cmd("detent", "卡位数量", "<count>", &cmd_detent);
    register_cmd("test", "电机快捷测试", "<spin|hold|detent|step>", &cmd_test);

    ESP_LOGI(TAG, "console ready — 输入 help / test spin");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
