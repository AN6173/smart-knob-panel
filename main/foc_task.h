#pragma once

/**
 * @file foc_task.h
 * @brief FOC 控制任务对外接口（控制台 / 日后 MQTT 都走这里调模式）。
 */

#include <stdbool.h>

#include "haptic/haptic_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bring-up / 运行时控制路径 */
typedef enum {
    FOC_CTRL_SENSOR = 0,   /**< 只读编码器，不出力矩（验 SPI） */
    FOC_CTRL_OPENLOOP,     /**< 开环速度（验 PWM / 电机接线） */
    FOC_CTRL_ANGLE,        /**< 闭环角度保持（验 initFOC） */
    FOC_CTRL_HAPTIC,       /**< 力矩触感（主模式） */
    FOC_CTRL_COUNT
} foc_ctrl_mode_t;

/** 创建并启动 FOC 任务（内部完成传感器 / 驱动 / initFOC） */
void foc_task_start(void);

void foc_set_ctrl_mode(foc_ctrl_mode_t mode);
foc_ctrl_mode_t foc_get_ctrl_mode(void);
const char *foc_ctrl_mode_name(foc_ctrl_mode_t mode);

void foc_set_haptic_mode(haptic_mode_t mode);
haptic_mode_t foc_get_haptic_mode(void);
/** 切触感模式并让当前轴角落在第 position 格中心（UI 回到菜单时保持选中项） */
void foc_set_haptic_mode_pos(haptic_mode_t mode, int32_t position);
/** 设置卡位数量并切到 detent 模式（2–64） */
void foc_set_detent_count(int32_t count);

/** openloop：目标角速度 rad/s；angle：目标轴角 rad */
void foc_set_target(float value);
float foc_get_target(void);

/** 遥测；指针可为 NULL。angle=轴角(可多圈), sensor=编码器单圈[0,2π)
 *  current = q 轴估算电流 (A)，(Uq − Ke·ω)/Rs */
void foc_get_telemetry(float *angle, float *velocity, float *current);

/** 电机电气遥测（估算）；指针可为 NULL */
typedef struct {
    float uq;          /**< 实际下发 Uq (V) */
    float i_est;       /**< 估算 q 轴电流 (A) */
    float i_rms;       /**< I²t 热模型 RMS 电流 (A) */
    float i_limit_eff; /**< 当前生效的峰值限流 (A，含热降额) */
    float p_copper;    /**< 估算铜损 3/2·I²·Rs (W，RMS) */
    bool sleeping;     /**< 空闲休眠中（EN=0） */
} foc_power_t;
void foc_get_power(foc_power_t *out);
void foc_get_position(float *shaft_rad, float *sensor_rad, float *velocity);

/** 串口实时打印角度（默认开启，约 10 Hz） */
void foc_set_angle_stream(bool enable);
bool foc_get_angle_stream(void);

/** 峰值电流上限 (A)，夹在 0.05 .. BOARD_I_LIMIT_ABS */
void foc_set_current_limit(float amps);
float foc_get_current_limit(void);

/** Uq 硬上限 (V)，额外保险；默认 = BOARD_I_LIMIT_ABS·Rs + 反电动势余量 */
void foc_set_voltage_limit(float vlim);
float foc_get_voltage_limit(void);

/** 供电电压 VM (V)，必须与实物一致 */
void foc_set_supply_voltage(float v);
float foc_get_supply_voltage(void);

/** 触感状态快照（UI 用）：驱动未就绪时也按编码器角度实时更新 */
typedef struct {
    haptic_mode_t mode;
    int32_t position;     /**< 当前格位 */
    int32_t num_positions;/**< 0 = 无界 */
    float value_norm;     /**< 有界模式 0–1 */
    float shaft_rad;      /**< 轴角（相对模式原点） */
} foc_haptic_snapshot_t;
void foc_get_haptic_snapshot(foc_haptic_snapshot_t *out);

/** FOC 任务主循环频率（Hz，1s 滑动统计；闭环时为 loopFOC 次数） */
float foc_get_loop_hz(void);

/** 清除 NVS 中的电角度校准，下次上电重新 initFOC */
void foc_clear_calibration(void);

/** 相位静态测试（诊断用，≤3V，每步 400ms，结束 EN=0） */
void foc_phase_test(float v);

/** 急停：回 sensor，力矩/目标清零 */
void foc_stop(void);

/**
 * 按键演示：短按切换下一档测试，长按急停。
 * 档位：停(sensor) → 自由 → 卡位 → 停 …
 */
void foc_demo_next(void);

/** 初始化流程是否结束（成功或失败都会置位，按键任务需等待） */
bool foc_is_ready(void);
/** initFOC 是否成功（失败时禁止出力） */
bool foc_is_ok(void);

#ifdef __cplusplus
}
#endif
