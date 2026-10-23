#pragma once

/**
 * @file haptic_mode.h
 * @brief 触感模式：根据轴角/速度计算力矩指令（电压量纲，交给 SimpleFOC torque 模式）。
 *
 * Free / Detent 已可用；Switch / Dimmer / Curtain 为可运行的 stub，参数待上板细调。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAPTIC_FREE = 0,    /**< 近零力矩，自由转动 */
    HAPTIC_DETENT,      /**< 离散卡位（如 12 / 24） */
    HAPTIC_SWITCH,      /**< 强回中到 0° 或 180° */
    HAPTIC_DIMMER,      /**< 软限位，映射 0–100%（stub） */
    HAPTIC_CURTAIN,     /**< 行程端点（stub） */
    HAPTIC_MODE_COUNT
} haptic_mode_t;

typedef struct {
    int32_t detent_count;       /**< 每圈卡位数 */
    float detent_strength;      /**< 卡位力矩强度 */
    float snap_point;           /**< 越过半宽多少比例时跳到下一格 */
    float endstop_strength;     /**< 端点力矩 */
    float range_rad;            /**< 有界模式总行程 (rad) */
} haptic_params_t;

typedef struct {
    haptic_mode_t mode;
    haptic_params_t params;
    int32_t position;           /**< 当前卡位 / 开关索引 */
    float value_norm;           /**< Dimmer/Curtain：0.0–1.0 */
} haptic_state_t;

void haptic_init(haptic_state_t *state);
void haptic_set_mode(haptic_state_t *state, haptic_mode_t mode);
const char *haptic_mode_name(haptic_mode_t mode);

/**
 * 由 FOC 任务每周期调用；禁止在此打日志 / 分配内存。
 * @return 力矩指令（与 motor.move 在 torque 模式下的量纲一致）
 */
float haptic_compute_torque(haptic_state_t *state, float shaft_angle, float shaft_velocity);

#ifdef __cplusplus
}
#endif
