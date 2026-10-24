/**
 * @file foc_task.cpp
 * @brief FOC 主循环：MT6701(SPI) + BLDC 3PWM，电压模式 FOC + 软件电流估算/限流 + 触感。
 *
 * 依赖 espressif/esp_simplefoc。不要启用 SimpleFOC Serial，会与 ESP-IDF console 抢 UART0。
 *
 * 发热治理要点（详见 docs/thermal.md）：
 *   1. 供电电压按实物配置（旧版 9V vs 实物 12V → 电压指令被放大 1.33 倍）
 *   2. 所有闭环模式统一走「电流指令 → Uq = I·Rs + Ke·ω」，|I| 严格限幅 + I²t 降额
 *   3. Ud ≡ 0（不设电感 → SimpleFOC 不加 d 轴前馈），电角度校准存 NVS
 *   4. 锁角改为无积分 PD（旧版速度环 I=4 积分饱和 → 持续保持电流）
 *   5. 静止且无出力 → 拉低 EN 休眠；转动立即唤醒
 *   6. 开环仅诊断：Uq = 0.2A×Rs，10s 自动停
 */

#include "foc_task.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "esp_simplefoc.h"
#include "mt6701_spi.h"

#include "board_config.h"

static const char *TAG = "foc";

/* ---------- 硬件对象（全局静态，生命周期 = 整个固件） ---------- */
/* EN 不交给 SimpleFOC（它把第4参当相使能），由本文件独占 GPIO21 */
static BLDCDriver3PWM s_driver =
    BLDCDriver3PWM(BOARD_PWM_A_GPIO, BOARD_PWM_B_GPIO, BOARD_PWM_C_GPIO);
/* 刻意不传 R/KV/L：保持 SimpleFOC 纯电压模式，Ud=0，电流估算在本文件显式完成 */
static BLDCMotor s_motor = BLDCMotor(BOARD_MOTOR_POLE_PAIRS);
/* 自研 MT6701 SSI 驱动（mode1 + CRC 诊断），构造：host,sclk,miso,mosi,cs */
static Mt6701Spi s_encoder = Mt6701Spi(BOARD_ENC_SPI_HOST,
                                       BOARD_ENC_SCK_GPIO,
                                       BOARD_ENC_MISO_GPIO,
                                       BOARD_ENC_MOSI_GPIO,
                                       BOARD_ENC_CS_GPIO);

/* 控制台/按键任务与 FOC 任务共享的状态，用自旋锁保护 */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static haptic_state_t s_haptic;
static foc_ctrl_mode_t s_ctrl = FOC_CTRL_SENSOR; /* 默认识别位置，不出力矩 */
static float s_target = 0.0f;
static bool s_foc_aligned = false;
static bool s_foc_ok = false;
static bool s_angle_stream = false; /* 默认关，避免刷屏抢掉 knob> 输入 */
/* 按键：0停 1自由 2卡位 */
static int s_demo_step = 0;

/* 电流/电压限制（运行时可调） */
static float s_supply_v = BOARD_V_SUPPLY_DEFAULT;
static float s_i_limit = BOARD_I_LIMIT_DEFAULT;
static float s_uq_cap = 0.0f; /* 在 motor_init 中按供电计算 */

/* 遥测（FOC 任务写，其它任务读；float 单字写入原子即可） */
static volatile float s_uq = 0.0f;
static volatile float s_i_est = 0.0f;
static volatile float s_i_rms = 0.0f;
static volatile float s_i_limit_eff = 0.0f;
static volatile bool s_sleeping = false;

/* 锁角 PD（无积分 → 无积分饱和 → 不会持续憋电流） */
static constexpr float ANGLE_KP = 2.0f;    /* A/rad：偏 0.1rad → 0.2A */
static constexpr float ANGLE_KD = 0.03f;   /* A/(rad/s) */
static constexpr float ANGLE_DEADBAND = 0.01f; /* rad，约 0.6°，静止零电流 */

static int64_t s_openloop_start_us = 0;

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

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/** 默认 Uq 硬上限：最大允许电流压降 + 可能的反电动势，且不超过 SVPWM 线性区 Vdc/√3 */
static float default_uq_cap(float vsup)
{
    const float cap = BOARD_I_LIMIT_ABS * BOARD_MOTOR_RS_OHM + 1.0f;
    return fminf(cap, vsup / 1.7320508f * 0.95f);
}

static void apply_supply(float v)
{
    v = clampf(v, BOARD_V_SUPPLY_MIN, BOARD_V_SUPPLY_MAX);
    s_supply_v = v;
    s_driver.voltage_power_supply = v;
    s_driver.voltage_limit = v; /* 驱动层只做物理上限；真正的限制在 Uq 计算 */
    s_uq_cap = default_uq_cap(v);
    s_motor.voltage_limit = s_uq_cap;
}

static void motor_coast(void)
{
    /* 先断 EN，再清 PWM —— 半桥立刻 Hi-Z */
    en_set(false);
    s_target = 0.0f;
    s_uq = 0.0f;
    s_i_est = 0.0f;
    s_motor.controller = MotionControlType::torque;
    s_motor.target = 0.0f;
    s_motor.disable();
    s_driver.setPwm(0, 0, 0);
    pwm_force_zero();
    en_set(false);
}

static void motor_arm(void)
{
    s_motor.voltage_limit = s_uq_cap;
    s_motor.controller = MotionControlType::torque;
    s_motor.target = 0.0f;
    en_set(true);
    s_motor.enable();
    s_sleeping = false;
}

/* ---------- 电角度校准持久化 ---------- */
static constexpr const char *NVS_NS = "foc";
static constexpr uint32_t CAL_MAGIC = 0x464F4301u; /* "FOC" v1 */

static bool cal_load(float *zero_el, int *dir)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint32_t magic = 0, zbits = 0;
    int32_t d = 0, pp = 0;
    bool ok = nvs_get_u32(h, "magic", &magic) == ESP_OK && magic == CAL_MAGIC &&
              nvs_get_i32(h, "pp", &pp) == ESP_OK && pp == BOARD_MOTOR_POLE_PAIRS &&
              nvs_get_u32(h, "zero", &zbits) == ESP_OK &&
              nvs_get_i32(h, "dir", &d) == ESP_OK && (d == 1 || d == -1);
    nvs_close(h);
    if (ok) {
        memcpy(zero_el, &zbits, sizeof(float));
        *dir = (int)d;
        ok = std::isfinite(*zero_el);
    }
    return ok;
}

static void cal_save(float zero_el, int dir)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    uint32_t zbits;
    memcpy(&zbits, &zero_el, sizeof(float));
    nvs_set_u32(h, "magic", CAL_MAGIC);
    nvs_set_i32(h, "pp", BOARD_MOTOR_POLE_PAIRS);
    nvs_set_u32(h, "zero", zbits);
    nvs_set_i32(h, "dir", dir);
    nvs_commit(h);
    nvs_close(h);
}

void foc_clear_calibration(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "calibration cleared — 重启后重新 initFOC");
}

/* ---------- 对外接口 ---------- */

void foc_set_ctrl_mode(foc_ctrl_mode_t mode)
{
    if (mode >= FOC_CTRL_COUNT || !s_foc_aligned) {
        return;
    }
    if (mode != FOC_CTRL_SENSOR && !s_foc_ok) {
        ESP_LOGW(TAG, "initFOC 未成功，拒绝出力");
        return;
    }
    switch (mode) {
    case FOC_CTRL_SENSOR:
        s_ctrl = mode;
        motor_coast();
        break;
    case FOC_CTRL_OPENLOOP:
        /* 开环 Uq≡voltage_limit：没有电流反馈，只能把电压压到 0.2A×Rs */
        motor_arm();
        s_motor.voltage_limit = BOARD_I_OPENLOOP * BOARD_MOTOR_RS_OHM;
        s_motor.controller = MotionControlType::velocity_openloop;
        s_target = BOARD_OPENLOOP_VEL;
        s_openloop_start_us = esp_timer_get_time();
        s_ctrl = mode;
        break;
    case FOC_CTRL_ANGLE:
        s_target = s_motor.shaft_angle;
        motor_arm();
        s_ctrl = mode;
        break;
    case FOC_CTRL_HAPTIC:
        taskENTER_CRITICAL(&s_lock);
        s_haptic.origin = s_motor.shaft_angle; /* 以当前角为原点，不突跳 */
        s_haptic.position = 0;
        taskEXIT_CRITICAL(&s_lock);
        motor_arm();
        s_ctrl = mode;
        break;
    default:
        break;
    }
    ESP_LOGI(TAG, "ctrl -> %s (enabled=%d Ilim=%.2fA Vsup=%.1fV)",
             foc_ctrl_mode_name(mode), (int)s_motor.enabled, s_i_limit, s_supply_v);
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
    taskENTER_CRITICAL(&s_lock);
    haptic_set_mode(&s_haptic, mode, s_motor.shaft_angle);
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "haptic -> %s", haptic_mode_name(mode));
}

haptic_mode_t foc_get_haptic_mode(void)
{
    return s_haptic.mode;
}

void foc_set_detent_count(int32_t count)
{
    taskENTER_CRITICAL(&s_lock);
    haptic_set_detent_count(&s_haptic, count, s_motor.shaft_angle);
    const float kp = s_haptic.params.detent_kp;
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "detent_count -> %ld (kp=%.2f A/rad)", (long)count, kp);
}

void foc_set_target(float value)
{
    s_target = value;
}

float foc_get_target(void)
{
    return s_target;
}

void foc_get_telemetry(float *angle, float *velocity, float *current)
{
    if (angle) {
        *angle = s_motor.shaft_angle;
    }
    if (velocity) {
        *velocity = s_motor.shaft_velocity;
    }
    if (current) {
        *current = s_i_est;
    }
}

void foc_get_power(foc_power_t *out)
{
    if (!out) {
        return;
    }
    out->uq = s_uq;
    out->i_est = s_i_est;
    out->i_rms = s_i_rms;
    out->i_limit_eff = s_i_limit_eff;
    out->p_copper = 1.5f * s_i_rms * s_i_rms * BOARD_MOTOR_RS_OHM;
    out->sleeping = s_sleeping;
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

void foc_set_current_limit(float amps)
{
    s_i_limit = clampf(amps, 0.05f, BOARD_I_LIMIT_ABS);
    ESP_LOGI(TAG, "Ilim -> %.2f A (Uq≈%.2f V 静态)", s_i_limit, s_i_limit * BOARD_MOTOR_RS_OHM);
}

float foc_get_current_limit(void)
{
    return s_i_limit;
}

void foc_set_voltage_limit(float vlim)
{
    s_uq_cap = clampf(vlim, 0.2f, default_uq_cap(s_supply_v));
    if (s_ctrl != FOC_CTRL_OPENLOOP) {
        s_motor.voltage_limit = s_uq_cap;
    }
    ESP_LOGI(TAG, "Uq cap -> %.2f V", s_uq_cap);
}

float foc_get_voltage_limit(void)
{
    return s_uq_cap;
}

void foc_set_supply_voltage(float v)
{
    apply_supply(v);
    ESP_LOGI(TAG, "Vsupply -> %.2f V (Uq cap %.2f V)", s_supply_v, s_uq_cap);
}

float foc_get_supply_voltage(void)
{
    return s_supply_v;
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

bool foc_is_ok(void)
{
    return s_foc_ok;
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
        printf(">>> 停机。短按 → 自由模式\n");
        break;
    case 1:
        foc_set_haptic_mode(HAPTIC_FREE);
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
        printf("\n>>> 自由模式 Ilim=%.2fA foc_ok=%d —— 指令恒 0，静止 %.1fs 后驱动休眠\n",
               s_i_limit, (int)s_foc_ok, BOARD_IDLE_MS / 1000.0f);
        break;
    case 2:
        foc_set_detent_count(12);
        foc_set_ctrl_mode(FOC_CTRL_HAPTIC);
        printf("\n>>> 卡位×12 Ilim=%.2fA —— 静止在格中心为 0 电流；再短按/长按 → 停\n",
               s_i_limit);
        break;
    default:
        foc_stop();
        break;
    }
}

/** SimpleFOC 调试输出 → printf（不能用 Serial，会和 console 抢 UART0） */
class PrintfSink : public Print {
public:
    size_t write(uint8_t c) override
    {
        putchar((int)c);
        return 1;
    }
};
static PrintfSink s_sfoc_print;

/** 编码器自检：对齐前连续读 20 次，打印角度/CRC，排除 SPI/磁铁问题 */
static bool encoder_sanity_check(void)
{
    const uint32_t ok0 = s_encoder.crc_ok_count();
    const uint32_t fail0 = s_encoder.crc_fail_count();
    float amin = 10.0f, amax = -10.0f;
    for (int i = 0; i < 20; i++) {
        s_encoder.update();
        const float a = s_encoder.getMechanicalAngle();
        amin = fminf(amin, a);
        amax = fmaxf(amax, a);
        if (i < 5) {
            printf("*** 编码器[%d] %.2f° raw=0x%06lX crc=%s\n", i, a * 57.29578f,
                   (unsigned long)s_encoder.last_raw(), s_encoder.last_crc_ok() ? "OK" : "BAD");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const uint32_t ok = s_encoder.crc_ok_count() - ok0;
    const uint32_t fail = s_encoder.crc_fail_count() - fail0;
    const bool good = s_encoder.bus() != Mt6701Bus::None && ok >= 18;
    printf("*** 编码器自检 bus=%s ok=%lu fail=%lu 静止抖动=%.2f° → %s\n",
           s_encoder.bus_name(), (unsigned long)ok, (unsigned long)fail,
           (amax - amin) * 57.29578f, good ? "OK" : "FAIL（查 CS/CLK/MISO 接线与磁铁）");
    return good;
}

/**
 * 校准失败后的诊断：在两个相差 90° 的电角度上各施加 Uq，看转子转了多少。
 * 期望 ≈ 90°/pp = 12.9° 机械角；≈0 → 没出力（EN/VM/相线/电压太低）；
 * 方向不定/数值乱 → 编码器问题。
 */
static void align_move_test(float v)
{
    s_motor.enable();
    s_motor.setPhaseVoltage(v, 0.0f, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(400));
    s_encoder.update();
    const float a0 = s_encoder.getMechanicalAngle();
    s_motor.setPhaseVoltage(v, 0.0f, _normalizeAngle(_3PI_2 + _PI_2));
    vTaskDelay(pdMS_TO_TICKS(400));
    s_encoder.update();
    const float a1 = s_encoder.getMechanicalAngle();
    s_motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    float d = (a1 - a0) * 57.29578f;
    if (d > 180.0f) {
        d -= 360.0f;
    } else if (d < -180.0f) {
        d += 360.0f;
    }
    printf("*** 诊断 Uq=%.2fV: 电角度 +90° → 机械 %+.2f°（期望 ±%.1f°）%s\n",
           v, d, 90.0f / BOARD_MOTOR_POLE_PAIRS,
           fabsf(d) < 1.0f ? " ← 转子没动：查 VM 供电 / EN / 三相线 / SimpleFOCmini" : "");
}

/**
 * 多点精校电角度零点（仅首次上电，结果存 NVS）。
 *
 * SimpleFOC 默认只在 3π/2 一个电角度对齐一次，齿槽力矩/摩擦会让转子停偏几个电角度，
 * 零点偏差 δ 会让一部分电压落到 d 轴：同样的力矩需要 1/cos(δ) 倍电流，且产生无用 d 轴电流。
 * 这里正、反各扫一整圈机械角（= pp 个电周期），每 1/8 电周期停稳采样一次，
 * 对 (dir·pp·θm − θe) 做圆周平均：正反向摩擦滞后相互抵消，齿槽误差被平均。
 */
static bool refine_zero_electric_angle(float v_align)
{
    const int pp = BOARD_MOTOR_POLE_PAIRS;
    const int dir = (int)s_motor.sensor_direction;
    const int samples_per_erev = 8;
    const float step = 0.02f;                       /* 电角度步进 rad */
    const float span = 2.0f * (float)M_PI * pp;     /* 一整圈机械角 */
    const float sample_every = 2.0f * (float)M_PI / samples_per_erev;
    float sum_s = 0.0f, sum_c = 0.0f;
    int n = 0;

    for (int pass = 0; pass < 2; pass++) {
        const float sgn = (pass == 0) ? 1.0f : -1.0f;
        float next_sample = sample_every;
        for (float t = 0.0f; t <= span + 1e-4f; t += step) {
            const float th = _3PI_2 + sgn * t;
            s_motor.setPhaseVoltage(v_align, 0.0f, _normalizeAngle(th));
            vTaskDelay(1);
            if (t >= next_sample) {
                next_sample += sample_every;
                vTaskDelay(pdMS_TO_TICKS(80)); /* 停稳 */
                s_encoder.update();
                const float mech = s_encoder.getMechanicalAngle();
                /* 与 SimpleFOC 定义一致：在 θe=3π/2 处 zero = dir·pp·θm */
                const float z = _normalizeAngle((float)(dir * pp) * mech - (th - _3PI_2));
                sum_s += sinf(z);
                sum_c += cosf(z);
                n++;
            }
        }
    }
    s_motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);

    const float r = sqrtf(sum_s * sum_s + sum_c * sum_c) / (float)(n > 0 ? n : 1);
    const float z = _normalizeAngle(atan2f(sum_s, sum_c));
    printf("*** 精校零点: %d 点, 一致性 R=%.3f (1=完美), zero_el %.4f → %.4f\n",
           n, r, s_motor.zero_electric_angle, z);
    if (n < 8 || r < 0.95f) {
        ESP_LOGW(TAG, "zero refine inconsistent (R=%.3f), keep SimpleFOC value", r);
        return false;
    }
    s_motor.zero_electric_angle = z;
    return true;
}

/* ---------- 初始化 ---------- */

static void motor_init(void)
{
    haptic_init(&s_haptic);

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

    apply_supply(BOARD_V_SUPPLY_DEFAULT);
    s_driver.init({BOARD_LEDC_CH_A, BOARD_LEDC_CH_B, BOARD_LEDC_CH_C});
    s_driver.setPwm(0, 0, 0);
    pwm_force_zero();
    en_set(false);
    s_motor.linkDriver(&s_driver);

    s_motor.foc_modulation = SpaceVectorPWM;
    /* 非居中调制：Uq=0 时三相均为 0 占空（下管常通），无共模开关损耗 */
    s_motor.modulation_centered = false;
    s_motor.controller = MotionControlType::torque;
    s_motor.torque_controller = TorqueControlType::voltage;
    s_motor.motion_downsample = 0;
    s_motor.velocity_limit = 20.0f;
    /* 速度滤波：5ms，阻尼项够干净又不拖慢手感 */
    s_motor.LPF_velocity.Tf = 0.005f;
    s_motor.LPF_angle.Tf = 0.0f;

    /* 打开 SimpleFOC 调试输出：initFOC 失败时能看到真实原因（[SFOC] 前缀行） */
    printf("[SFOC] debug on\n");
    SimpleFOCDebug::enable(&s_sfoc_print);

    const bool enc_ok = encoder_sanity_check();

    float zero_el = 0.0f;
    int dir = 0;
    const bool have_cal = cal_load(&zero_el, &dir);

    float v_align = BOARD_V_ALIGN;
    s_motor.voltage_sensor_align = v_align;
    /* init() 会把 voltage_sensor_align 夹到 voltage_limit 以下 → 先放到上限档 */
    s_motor.voltage_limit = BOARD_V_ALIGN_MAX;

    en_set(true);
    vTaskDelay(pdMS_TO_TICKS(30));
    s_motor.init();
    s_motor.enable();

    int foc_ok = 0;
    if (have_cal) {
        /* 已有校准：SimpleFOC 见到方向 + 零角均已设置会跳过对齐（不再转动、不再大电流） */
        s_motor.zero_electric_angle = zero_el;
        s_motor.sensor_direction = (dir > 0) ? Direction::CW : Direction::CCW;
        printf("*** 使用 NVS 校准 zero_el=%.4f dir=%d（console: recal 可重新校准）\n", zero_el, dir);
        foc_ok = s_motor.initFOC();
    }

    /* 完整校准：逐档升压重试，每次重置方向/零点（NVS 数据可能失效） */
    bool fresh = false;
    while (!foc_ok && enc_ok && v_align <= BOARD_V_ALIGN_MAX + 1e-3f) {
        s_motor.sensor_direction = Direction::UNKNOWN;
        s_motor.zero_electric_angle = NOT_SET;
        s_motor.voltage_sensor_align = v_align;
        s_motor.enable(); /* 上次失败 initFOC 内部会 disable() */
        printf("*** initFOC 校准中 Uq=%.2fV(≈%.2fA)（电机会轻动，手勿碰轴）EN=%d …\n",
               v_align, v_align / BOARD_MOTOR_RS_OHM, gpio_get_level(BOARD_DRIVER_ENABLE_GPIO));
        foc_ok = s_motor.initFOC();
        fresh = true;
        if (!foc_ok) {
            v_align += BOARD_V_ALIGN_STEP;
        }
    }

    if (foc_ok && fresh) {
        if (!s_motor.pp_check_result) {
            ESP_LOGW(TAG, "PP check fail：极对数可能不是 %d，仍保存但请核对", BOARD_MOTOR_POLE_PAIRS);
        }
        s_motor.enable();
        refine_zero_electric_angle(s_motor.voltage_sensor_align);
        cal_save(s_motor.zero_electric_angle, (int)s_motor.sensor_direction);
    } else if (!foc_ok && enc_ok) {
        align_move_test(BOARD_V_ALIGN_MAX);
    }
    s_motor.voltage_limit = s_uq_cap;
    motor_coast();

    if (foc_ok) {
        s_foc_ok = true;
        printf("\n*** initFOC OK  dir=%d zero_el=%.4f\n",
               (int)s_motor.sensor_direction, s_motor.zero_electric_angle);
    } else {
        /* 零角未知时任何 Uq 都会落在错误的 d/q 上 → 纯发热不出力；禁止上电 */
        s_foc_ok = false;
        printf("\n*** initFOC FAIL —— 禁止出力（电角度未知会产生 d 轴电流 → 发热）\n"
               "*** 原因见上方 [SFOC] 行 / 编码器自检 / 诊断行；Mini.EN 应接 GPIO%d\n",
               (int)BOARD_DRIVER_ENABLE_GPIO);
        ESP_LOGW(TAG, "initFOC failed");
    }

    s_ctrl = FOC_CTRL_SENSOR;
    s_foc_aligned = true; /* 初始化流程结束（成功与否由 s_foc_ok 区分） */
    ESP_LOGI(TAG, "motor init Vsup=%.1fV Ilim=%.2fA Irms=%.2fA Uqcap=%.2fV foc_ok=%d",
             s_supply_v, s_i_limit, BOARD_I_RMS_LIMIT, s_uq_cap, foc_ok);
    printf("*** 短按：停→自由→卡位→停。长按急停。\n\n");
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

    while (true) {
        if (s_angle_stream) {
            float shaft = 0.0f, sens = 0.0f, vel = 0.0f;
            foc_get_position(&shaft, &sens, &vel);
            const float deg = sens * rad2deg;
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
            printf("\rpos %6.1f° [%s] raw=0x%06lX crc=%s ok=%lu fail=%lu%s | shaft %+6.2f I=%+.3fA  ",
                   deg,
                   s_encoder.bus_name(),
                   (unsigned long)raw,
                   s_encoder.last_crc_ok() ? "OK " : "BAD",
                   (unsigned long)s_encoder.crc_ok_count(),
                   (unsigned long)s_encoder.crc_fail_count(),
                   hint,
                   shaft, (float)s_i_est);
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------- 控制核心 ---------- */

/* 每次迭代的状态 */
static int64_t s_last_us = 0;
static float s_i2_filt = 0.0f;      /* I² 一阶滤波（热模型） */
static int64_t s_idle_since_us = 0;
static float s_sleep_angle = 0.0f;

/**
 * 电流指令 → Uq，严格限流。
 *   I_est = (Uq − Ke·ω)/Rs  ⇒  Uq = clamp(I) ·Rs + Ke·ω
 * 反电动势补偿让「零电流」真的为零：快拧自由旋钮时不再短路制动产生 Ke·ω/Rs 的电流。
 */
static float current_to_uq(float i_cmd, float omega, float i_lim)
{
    const float i = clampf(i_cmd, -i_lim, i_lim);
    const float bemf = BOARD_BEMF_COMP * BOARD_MOTOR_KE_PHASE * omega;
    float uq = i * BOARD_MOTOR_RS_OHM + bemf;
    uq = clampf(uq, -s_uq_cap, s_uq_cap);
    return uq;
}

static void control_step(void)
{
    const int64_t now = esp_timer_get_time();
    float dt = (s_last_us == 0) ? 0.0f : (float)(now - s_last_us) * 1e-6f;
    s_last_us = now;
    if (dt > 0.05f) {
        dt = 0.05f;
    }

    /* 1) 读编码器 + 下发上一拍的 Uq（Ud 恒 0） */
    s_motor.loopFOC();

    const float angle = s_motor.shaft_angle;
    const float vel = s_motor.shaft_velocity;

    /* 2) 计算电流指令 */
    float i_cmd = 0.0f;
    switch (s_ctrl) {
    case FOC_CTRL_ANGLE: {
        float err = s_target - angle;
        if (fabsf(err) < ANGLE_DEADBAND) {
            err = 0.0f;
        } else {
            err -= copysignf(ANGLE_DEADBAND, err);
        }
        i_cmd = ANGLE_KP * err - ANGLE_KD * vel;
        break;
    }
    case FOC_CTRL_HAPTIC:
        taskENTER_CRITICAL(&s_lock);
        i_cmd = haptic_compute_current(&s_haptic, angle, vel);
        taskEXIT_CRITICAL(&s_lock);
        break;
    default:
        break;
    }

    /* 3) I²t 热降额：RMS 超过上限时按比例收紧峰值 */
    const float i_rms = sqrtf(s_i2_filt);
    float i_lim = s_i_limit;
    if (i_rms > BOARD_I_RMS_LIMIT) {
        i_lim *= BOARD_I_RMS_LIMIT / i_rms;
    }
    s_i_limit_eff = i_lim;

    /* 4) 空闲休眠：静止 + 无出力 → EN=0；转动 > 2° 或出现出力需求即唤醒 */
    if (s_ctrl == FOC_CTRL_HAPTIC || s_ctrl == FOC_CTRL_ANGLE) {
        const bool quiet = fabsf(vel) < BOARD_IDLE_VEL && fabsf(i_cmd) < BOARD_IDLE_I;
        if (s_sleeping) {
            if (fabsf(angle - s_sleep_angle) > BOARD_WAKE_ANGLE || fabsf(i_cmd) >= BOARD_IDLE_I) {
                s_sleeping = false;
                s_idle_since_us = now;
                en_set(true);
                s_motor.enable();
            }
        } else if (!quiet) {
            s_idle_since_us = now;
        } else if (now - s_idle_since_us > (int64_t)BOARD_IDLE_MS * 1000) {
            s_sleeping = true;
            s_sleep_angle = angle;
            s_motor.move(0.0f);
            s_motor.disable();
            pwm_force_zero();
            en_set(false);
        }
    }

    float i_est = 0.0f;
    if (s_ctrl == FOC_CTRL_OPENLOOP) {
        /* 开环：Uq 恒为 voltage_limit，按静止最坏情况估算 */
        s_motor.move(s_target);
        s_uq = s_motor.voltage_limit;
        i_est = s_motor.voltage_limit / BOARD_MOTOR_RS_OHM;
        if (now - s_openloop_start_us > (int64_t)BOARD_OPENLOOP_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "openloop timeout → stop");
            foc_stop();
        }
    } else if (!s_sleeping) {
        const float uq = current_to_uq(i_cmd, vel, i_lim);
        s_motor.move(uq);
        s_uq = uq;
        i_est = (uq - BOARD_MOTOR_KE_PHASE * vel) / BOARD_MOTOR_RS_OHM;
    } else {
        s_motor.move(0.0f); /* disabled 时只刷新 shaft_angle/velocity，不出力 */
        s_uq = 0.0f;
        i_est = 0.0f; /* Hi-Z：无电流 */
    }
    s_i_est = i_est;

    /* 5) 热模型更新：τ = BOARD_I_RMS_TAU_S */
    if (dt > 0.0f) {
        const float a = dt / (BOARD_I_RMS_TAU_S + dt);
        s_i2_filt += a * (i_est * i_est - s_i2_filt);
        s_i_rms = sqrtf(s_i2_filt);
    }
}

static void foc_loop_task(void *arg)
{
    (void)arg;
    motor_init();

    while (true) {
        /* 停机/校准失败：只刷传感器；周期性再拉低 EN，防止被别的路径打开 */
        if (s_ctrl == FOC_CTRL_SENSOR || !s_foc_ok) {
            if (gpio_get_level(BOARD_DRIVER_ENABLE_GPIO) != 0) {
                en_set(false);
                pwm_force_zero();
            }
            if (s_encoder.bus() != Mt6701Bus::None) {
                s_encoder.update();
                /* 让 shaft_angle 跟随，切入闭环时原点正确 */
                s_motor.shaft_angle = s_motor.shaftAngle();
            }
            s_i_est = 0.0f;
            s_uq = 0.0f;
            s_sleeping = false;
            s_i2_filt *= 0.999f; /* 停机时热模型自然冷却 */
            s_i_rms = sqrtf(s_i2_filt);
            s_last_us = 0;
            vTaskDelay(1);
            continue;
        }

        for (int k = 0; k < BOARD_FOC_ITER_PER_TICK; k++) {
            control_step();
        }
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
