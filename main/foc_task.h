#pragma once

/**
 * @file foc_task.h
 * @brief FOC 控制任务对外接口（控制台 / 日后 MQTT 都走这里调模式）。
 */

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
/** 设置卡位数量并切到 detent 模式（2–64） */
void foc_set_detent_count(int32_t count);

/** openloop：目标角速度 rad/s；angle：目标轴角 rad */
void foc_set_target(float value);
float foc_get_target(void);

/** 遥测；指针可为 NULL。angle=轴角(可多圈), sensor=编码器单圈[0,2π) */
void foc_get_telemetry(float *angle, float *velocity, float *torque);
void foc_get_position(float *shaft_rad, float *sensor_rad, float *velocity);

/** 串口实时打印角度（默认开启，约 10 Hz） */
void foc_set_angle_stream(bool enable);
bool foc_get_angle_stream(void);

/** 运行时限压（V），夹在 0.5 .. BOARD_V_SUPPLY */
void foc_set_voltage_limit(float vlim);
float foc_get_voltage_limit(void);

/** 急停：回 sensor，力矩/目标清零 */
void foc_stop(void);

/**
 * 按键演示：短按切换下一档测试，长按急停。
 * 档位：停(sensor) → 开环慢转 → 闭环锁角 → 卡位 → 停 …
 */
void foc_demo_next(void);

/** FOC 是否已完成 initFOC（按键任务需等待） */
bool foc_is_ready(void);

#ifdef __cplusplus
}
#endif
