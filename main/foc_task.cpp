/**
 * @file foc_task.cpp
 * @brief FOC 主循环：MT6701(SPI) + BLDC 3PWM + 开环/闭环/触感切换。
 *
 * 依赖 espressif/esp_simplefoc。不要启用 SimpleFOC Serial，会与 ESP-IDF console 抢 UART0。
 */

#include "foc_task.h"

#include <cstdio>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_simplefoc.h"
#include "mt6701_spi.h"

#include "board_config.h"

static const char *TAG = "foc";

/* ---------- 硬件对象（全局静态，生命周期 = 整个固件） ---------- */
/* EN 不交给 SimpleFOC（它把第4参当相使能），由本文件独占 GPIO21 */
static BLDCDriver3PWM s_driver =
    BLDCDriver3PWM(BOARD_PWM_A_GPIO, BOARD_PWM_B_GPIO, BOARD_PWM_C_GPIO);
static BLDCMotor s_motor = BLDCMotor(BOARD_MOTOR_POLE_PAIRS);
/* 自研 MT6701 SSI 驱动（mode1 + CRC 诊断），构造：host,sclk,miso,mosi,cs */
static Mt6701Spi s_encoder = Mt6701Spi(BOARD_ENC_SPI_HOST,
                                       BOARD_ENC_SCK_GPIO,
                                       BOARD_ENC_MISO_GPIO,
                                       BOARD_ENC_MOSI_GPIO,
                                       BOARD_ENC_CS_GPIO);

static haptic_state_t s_haptic;
static foc_ctrl_mode_t s_ctrl = FOC_CTRL_SENSOR; /* 默认识别位置，不出力矩 */
static float s_target = 0.0f;
static float s_last_torque = 0.0f;
static bool s_foc_aligned = false;
static bool s_foc_ok = false;
static bool s_angle_stream = false; /* 默认关，避免刷屏抢掉 knob> 输入 */
/* 按键：0停 1自由 2卡位（均可一直跑；开环不进主循环） */
static int s_demo_step = 0;

static bool s_en_ready = false;

static void en_gpio_prepare(gpio_num_t pin)
{
    if (rtc_gpio_is_valid_gpio(pin)) {
        rtc_gpio_deinit(pin);
    }
    gpio_reset_pin(pin);
    /* S3 上 get_level 读焊盘真实电平；INPUT_OUTPUT 才能可靠回读 */
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
}

static void en_set(bool on)
{
    if (!s_en_ready) {
        en_gpio_prepare(BOARD_DRIVER_ENABLE_GPIO);
        s_en_ready = true;
    }
    gpio_set_level(BOARD_DRIVER_ENABLE_GPIO, on ? 1 : 0);
}


static void pwm_force_zero(void)
{
    /* 直接把 LEDC 占空比打到 0，避免 disable 后仍有残留开关 */
    const ledc_channel_t chs[] = {
        (ledc_channel_t)BOARD_LEDC_CH_A,
        (ledc_channel_t)BOARD_LEDC_CH_B,
        (ledc_channel_t)BOARD_LEDC_CH_C,
    };
    for (ledc_channel_t ch : chs) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }
}

static void motor_coast(void)
{
    /* 先断 EN，再清 PWM —— 半桥立刻 Hi-Z */
    en_set(false);
    s_target = 0.0f;
    s_last_torque = 0.0f;
    s_motor.controller = MotionControlType::torque;
    s_motor.move(0);
    s_motor.disable();
    s_driver.setPwm(0, 0, 0);
    pwm_force_zero();
    en_set(false);
}

static void apply_voltage_limit(float vlim)
{
    if (vlim < 0.5f) {
        vlim = 0.5f;
    }
    if (vlim > BOARD_V_SUPPLY) {
        vlim = BOARD_V_SUPPLY;
    }
    s_driver.voltage_limit = vlim;
    s_motor.voltage_limit = vlim;
}

static void motor_arm(float vlim)
{
    apply_voltage_limit(vlim);
    en_set(true);
    s_motor.enable();
}

void foc_set_ctrl_mode(foc_ctrl_mode_t mode)
{
    if (mode >= FOC_CTRL_COUNT) {
        return;
    }
    s_ctrl = mode;
    switch (mode) {
    case FOC_CTRL_SENSOR:
        motor_coast();
        break;
    case FOC_CTRL_OPENLOOP:
        /* 开环 Uq≡voltage_limit，用更低限压防烫 */
        motor_arm(BOARD_V_OPENLOOP);
        s_motor.controller = MotionControlType::velocity_openloop;
        s_target = BOARD_OPENLOOP_VEL;
        break;
    case FOC_CTRL_ANGLE:
        motor_arm(BOARD_V_LIMIT);
        s_motor.controller = MotionControlType::angle;
        s_target = s_motor.shaft_angle;
        break;
    case FOC_CTRL_HAPTIC:
        motor_arm(BOARD_V_LIMIT);
        s_motor.controller = MotionControlType::torque;
        break;
    default:
        break;
    }
    ESP_LOGI(TAG, "ctrl -> %s (enabled=%d Vlim=%.1f)",
             foc_ctrl_mode_name(mode), (int)s_motor.enabled, s_motor.voltage_limit);
}

foc_ctrl_mode_t foc_get_ctrl_mode(void)
{
    return s_ctrl;
}

const char *foc_ctrl_mode_name(foc_ctrl_mode_t mode)
{
    switch (mode) {
    case FOC_CTRL_SENSOR:
        return "sensor";
    case FOC_CTRL_OPENLOOP:
        return "openloop";
    case FOC_CTRL_ANGLE:
        return "angle";
    case FOC_CTRL_HAPTIC:
        return "haptic";
    default:
        return "unknown";
    }
}

void foc_set_haptic_mode(haptic_mode_t mode)
{
    haptic_set_mode(&s_haptic, mode);
    ESP_LOGI(TAG, "haptic -> %s", haptic_mode_name(mode));
}

haptic_mode_t foc_get_haptic_mode(void)
{
    return s_haptic.mode;
}

void foc_set_detent_count(int32_t count)
{
    if (count < 2) {
        count = 2;
    }
    if (count > 64) {
        count = 64;
    }
    s_haptic.params.detent_count = count;
    if (s_haptic.mode != HAPTIC_DETENT) {
        foc_set_haptic_mode(HAPTIC_DETENT);
        s_haptic.params.detent_count = count; /* set_mode 会重置为 12，再写回 */
    }
    ESP_LOGI(TAG, "detent_count -> %ld", (long)count);
}

void foc_set_target(float value)
{
    s_target = value;
}

float foc_get_target(void)
{
    return s_target;
}

void foc_get_telemetry(float *angle, float *velocity, float *torque)
{
    if (angle) {
        *angle = s_motor.shaft_angle;
    }
    if (velocity) {
        *velocity = s_motor.shaft_velocity;
    }
    if (torque) {
        *torque = s_last_torque;
    }
}

void foc_get_position(float *shaft_rad, float *sensor_rad, float *velocity)
{
    if (shaft_rad) {
        *shaft_rad = s_motor.shaft_angle;
    }
    if (sensor_rad) {
        /* 只用缓存，勿再打 SPI —— 与 FOC 环并发会读出 0xFFFFFF/0x000000 乱跳 */
        *sensor_rad = s_encoder.cached_angle();
    }
    if (velocity) {
        *velocity = s_motor.shaft_velocity;
    }
}

void foc_set_angle_stream(bool enable)
{
    s_angle_stream = enable;
    ESP_LOGI(TAG, "angle stream %s", enable ? "ON" : "OFF");
}

bool foc_get_angle_stream(void)
{
    return s_angle_stream;
}

void foc_set_voltage_limit(float vlim)
{
    apply_voltage_limit(vlim);
    ESP_LOGI(TAG, "Vlim -> %.1f V", vlim);
}

float foc_get_voltage_limit(void)
{
    return s_motor.voltage_limit;
}

void foc_stop(void)
{
    s_demo_step = 0;
    s_ctrl = FOC_CTRL_SENSOR;
    motor_coast();
    printf("\n>>> [STOP] EN=%d 已关断\n", gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
    ESP_LOGI(TAG, "STOP EN=%d", gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
}

bool foc_is_ready(void)
{
    return s_foc_aligned;
}

void foc_demo_next(void)
{
    if (!s_foc_aligned) {
        printf("\n>>> FOC 未就绪，稍后再按\n");
        return;
    }

    foc_set_angle_stream(false);
    s_demo_step = (s_demo_step + 1) % 3;

    switch (s_demo_step) {
    case 0:
        foc_stop();
        printf(">>> 停机。短按 → 自由模式（可一直跑，空闲力矩≈0）\n");
        break;
    case 1:
        foc_set_haptic_mode(HAPTIC_FREE);
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
        printf("\n>>> 自由模式（一直运行）Vlim=%.1fV EN=%d foc_ok=%d\n"
               "    空闲指令≈0，不烫；拧动应很轻。再短按 → 卡位\n",
               foc_get_voltage_limit(), gpio_get_level(BOARD_DRIVER_ENABLE_GPIO),
               (int)s_foc_ok);
        break;
    case 2:
        foc_set_detent_count(12);
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
        printf("\n>>> 卡位×12（一直运行）Vlim=%.1fV —— 静止在齿槽中心时几乎不出力\n"
               "    再短按/长按 → 停\n",
               foc_get_voltage_limit());
        break;
    default:
        foc_stop();
        break;
    }
}

static void motor_init(void)
{
    haptic_init(&s_haptic);
    haptic_set_mode(&s_haptic, HAPTIC_FREE);

    en_set(false);
    en_set(true);
    const int en_hi = gpio_get_level(BOARD_DRIVER_ENABLE_GPIO);
    en_set(false);
    const int en_lo = gpio_get_level(BOARD_DRIVER_ENABLE_GPIO);
    printf("*** EN GPIO%d 自检: 写1读%d 写0读%d %s\n",
           (int)BOARD_DRIVER_ENABLE_GPIO, en_hi, en_lo,
           (en_hi == 1 && en_lo == 0) ? "OK" : "FAIL");
    if (en_hi != 1 || en_lo != 0) {
        ESP_LOGE(TAG, "EN GPIO%d self-test failed", (int)BOARD_DRIVER_ENABLE_GPIO);
    }

    s_encoder.init();
    ESP_LOGI(TAG, "encoder bus=%s", s_encoder.bus_name());
    s_motor.linkSensor(&s_encoder);

    s_driver.voltage_power_supply = BOARD_V_SUPPLY;
    s_driver.voltage_limit = BOARD_V_LIMIT;
    s_driver.init({BOARD_LEDC_CH_A, BOARD_LEDC_CH_B, BOARD_LEDC_CH_C});
    s_driver.setPwm(0, 0, 0);
    pwm_force_zero();
    en_set(false);
    s_motor.linkDriver(&s_driver);

    s_motor.voltage_limit = BOARD_V_LIMIT;
    s_motor.velocity_limit = 10.0f;
    s_motor.foc_modulation = SpaceVectorPWM;
    /* Uq=0 时不要把三相顶到 Vlim/2 空切换 —— 减轻驱动发热 */
    s_motor.modulation_centered = false;
    s_motor.controller = MotionControlType::torque;
    s_motor.torque_controller = TorqueControlType::voltage;

    s_motor.PID_velocity.P = 0.4f;
    s_motor.PID_velocity.I = 4.0f;
    s_motor.PID_velocity.D = 0.0f;
    s_motor.PID_velocity.output_ramp = 800.0f;
    s_motor.LPF_velocity.Tf = 0.01f;

    s_motor.P_angle.P = 8.0f;
    s_motor.P_angle.I = 0.0f;
    s_motor.P_angle.D = 0.0f;

    s_motor.voltage_sensor_align = BOARD_V_ALIGN;

    /* 校准：EN=1，align 电压不被 V_LIMIT 卡住；失败则用默认值仍可跑自由模式 */
    s_driver.voltage_limit = BOARD_V_ALIGN;
    s_motor.voltage_limit = BOARD_V_ALIGN;
    en_set(true);
    vTaskDelay(pdMS_TO_TICKS(30));
    s_motor.init();
    s_motor.enable();
    en_set(true);
    vTaskDelay(pdMS_TO_TICKS(30));
    printf("*** initFOC 校准中（电机会轻动）EN=%d …\n",
           gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
    int foc_ok = s_motor.initFOC();
    if (!foc_ok) {
        /* 再试一次：有时首次对齐角度不够 */
        en_set(true);
        s_motor.enable();
        s_motor.sensor_direction = Direction::UNKNOWN;
        s_motor.zero_electric_angle = NOT_SET;
        foc_ok = s_motor.initFOC();
    }
    apply_voltage_limit(BOARD_V_LIMIT);
    motor_coast();

    if (foc_ok) {
        s_foc_ok = true;
        printf("\n*** initFOC OK  dir=%d zero_el=%.3f —— 可长期闭环\n",
               (int)s_motor.sensor_direction, s_motor.zero_electric_angle);
    } else {
        s_foc_ok = false;
        s_motor.sensor_direction = Direction::CW;
        s_motor.zero_electric_angle = 0;
        printf("\n*** initFOC FAIL —— 仍可短按进「自由」(空闲力矩0，应不烫)\n"
               "*** 卡位手感可能差。查: Mini.EN 是否真接 GPIO%d，停机时 EN 应为 0V\n",
               (int)BOARD_DRIVER_ENABLE_GPIO);
        ESP_LOGW(TAG, "initFOC failed");
    }

    s_foc_aligned = true;
    s_ctrl = FOC_CTRL_SENSOR;
    ESP_LOGI(TAG, "motor init supply=%.0f Vlim=%.1f foc_ok=%d EN=%d",
             BOARD_V_SUPPLY, BOARD_V_LIMIT, foc_ok,
             gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
    printf("*** 短按：停→自由(一直跑)→卡位(一直跑)→停。长按急停。\n"
           "*** 不要用开环长时间转；开环=持续电流=必烫（换库也一样）。\n\n");
}

/** 串口实时位置：单圈角度 + 多圈轴角 + 速度（约 10 Hz） */
static void angle_stream_task(void *arg)
{
    (void)arg;
    const float rad2deg = 180.0f / 3.14159265f;

    /* 等 FOC 初始化完成再开始刷屏 */
    while (!s_foc_aligned) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("\n--- position stream (stream off 关闭) ---\n");

    while (true) {
        if (s_angle_stream) {
            float shaft = 0.0f, sens = 0.0f, vel = 0.0f;
            foc_get_position(&shaft, &sens, &vel);
            const float deg = sens * rad2deg;
            const float turns = shaft / (2.0f * 3.14159265f);
            const uint32_t raw = s_encoder.last_raw();
            const char *hint = "";
            if (s_encoder.bus() == Mt6701Bus::None) {
                hint = " << NO_BUS";
            } else if (s_encoder.bus() == Mt6701Bus::Spi && raw == 0xFFFFFFu) {
                hint = " << OPEN";
            } else if (s_encoder.bus() == Mt6701Bus::Spi && raw == 0u) {
                hint = " << STUCK0";
            } else if (!s_encoder.last_crc_ok()) {
                hint = " << BAD";
            }
            printf("\rpos %6.1f° [%s] raw=0x%06lX crc=%s ok=%lu fail=%lu%s | shaft %+6.2f  ",
                   deg,
                   s_encoder.bus_name(),
                   (unsigned long)raw,
                   s_encoder.last_crc_ok() ? "OK " : "BAD",
                   (unsigned long)s_encoder.crc_ok_count(),
                   (unsigned long)s_encoder.crc_fail_count(),
                   hint,
                   shaft);
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void foc_loop_task(void *arg)
{
    (void)arg;
    motor_init();

    while (true) {
        if (!s_foc_aligned) {
            vTaskDelay(1);
            continue;
        }

        /* 停机：只刷传感器；周期性再拉低 EN，防止被别的路径打开 */
        if (s_ctrl == FOC_CTRL_SENSOR || !s_motor.enabled) {
            if (gpio_get_level(BOARD_DRIVER_ENABLE_GPIO) != 0) {
                en_set(false);
                pwm_force_zero();
            }
            if (s_encoder.bus() != Mt6701Bus::None) {
                s_encoder.update();
            }
            s_last_torque = 0.0f;
            vTaskDelay(1);
            continue;
        }

        s_motor.loopFOC();

        float cmd = 0.0f;
        switch (s_ctrl) {
        case FOC_CTRL_OPENLOOP:
        case FOC_CTRL_ANGLE:
            cmd = s_target;
            break;
        case FOC_CTRL_HAPTIC:
            cmd = haptic_compute_torque(&s_haptic, s_motor.shaft_angle, s_motor.shaft_velocity);
            break;
        default:
            cmd = 0.0f;
            break;
        }

        s_last_torque = cmd;
        s_motor.move(cmd);
        vTaskDelay(1);
    }
}

void foc_task_start(void)
{
    xTaskCreatePinnedToCore(foc_loop_task,
                            "foc",
                            BOARD_FOC_TASK_STACK,
                            nullptr,
                            BOARD_FOC_TASK_PRIORITY,
                            nullptr,
                            BOARD_FOC_TASK_CORE);

    /* 位置打印放在另一核，避免拖慢 FOC 环 */
    xTaskCreatePinnedToCore(angle_stream_task,
                            "pos_stream",
                            3072,
                            nullptr,
                            2,
                            nullptr,
                            0);
}
