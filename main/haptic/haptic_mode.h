#pragma once

/**
 * @file haptic_mode.h
 * @brief 触感模式（SmartKnob 思路）：根据轴角/速度计算 q 轴电流指令（单位 A）。
 *
 * 输出是「电流」而非电压：foc_task 再按 Uq = I·Rs + Ke·ω 换算并做严格限流，
 * 因此这里的强度参数与供电电压无关，换 7.4V/12V 手感一致。
 *
 * 统一模型：一串宽度为 width 的虚拟格位（可有界），
 *   - 格内：死区外的线性弹簧把轴拉向格中心（静止在中心时电流=0，不发热）
 *   - 越过 snap 点：格位 +1/-1，力矩方向翻转 → 「咔哒」感
 *   - 有界模式越界：更硬的端点弹簧
 *   - 运动阻尼：仅在转动时出力
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAPTIC_FREE = 0,    /**< 零力矩，自由转动 */
    HAPTIC_DETENT,      /**< 离散卡位（如 12 / 24），无界 */
    HAPTIC_SWITCH,      /**< 双稳态开关（2 位，有界） */
    HAPTIC_DIMMER,      /**< 调光 0–100%（270°，细腻无卡位 + 软端点） */
    HAPTIC_CURTAIN,     /**< 窗帘 0–100%（一圈 20 格 + 端点） */
    HAPTIC_MODE_COUNT
} haptic_mode_t;

typedef struct {
    float width_rad;            /**< 每格角宽 (rad) */
    int32_t num_positions;      /**< 格位数；0 = 无界 */
    float detent_kp;            /**< 卡位弹簧 A/rad */
    float endstop_kp;           /**< 端点弹簧 A/rad */
    float damping;              /**< 阻尼 A/(rad/s) */
    float snap_point;           /**< 越过 width×snap 换格（>0.5 带迟滞） */
    float dead_zone;            /**< 格中心死区占 width 比例（静止零电流） */
    float i_peak;               /**< 本模式电流上限 A（再受全局限流约束） */
} haptic_params_t;

typedef struct {
    haptic_mode_t mode;
    haptic_params_t params;
    float origin;               /**< 第 0 格中心对应的轴角 */
    int32_t position;           /**< 当前格位 */
    float value_norm;           /**< 有界模式：0.0–1.0 */
} haptic_state_t;

void haptic_init(haptic_state_t *state);
/** 切模式并以 shaft_angle 为新原点（不会突跳） */
void haptic_set_mode(haptic_state_t *state, haptic_mode_t mode, float shaft_angle);
/** 修改卡位数（DETENT），保持当前角为格中心 */
void haptic_set_detent_count(haptic_state_t *state, int32_t count, float shaft_angle);
const char *haptic_mode_name(haptic_mode_t mode);

/**
 * 由 FOC 任务每周期调用；禁止在此打日志 / 分配内存。
 * @return q 轴电流指令 (A)
 */
float haptic_compute_current(haptic_state_t *state, float shaft_angle, float shaft_velocity);

#ifdef __cplusplus
}
#endif
