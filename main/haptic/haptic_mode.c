/**
 * @file haptic_mode.c
 * @brief 触感电流计算（参考 scottbez1/smartknob 的格位 + snap + 死区模型）。
 *
 * 参数整定依据（2804, Rs=2.5Ω, 12 卡位）：
 *   格宽 30° = 0.524 rad，snap 0.55 → 最大偏离 ≈ 0.29 rad
 *   kp = 1.1 A/rad → 峰值 ≈ 0.32A（≈0.8V），手指明显能感到「咔哒」
 *   静止时落在死区内 → I = 0；只有拧动的那一下才有电流 → 平均电流极低
 */

#include "haptic_mode.h"

#include <math.h>
#include <stdbool.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void params_for_mode(haptic_params_t *p, haptic_mode_t mode)
{
    /* 公共默认 */
    p->snap_point = 0.55f;
    p->dead_zone = 0.04f;
    p->endstop_kp = 2.0f;
    p->damping = 0.003f;
    p->i_peak = 0.35f;

    switch (mode) {
    case HAPTIC_FREE:
        p->width_rad = 2.0f * PI / 36.0f; /* 仅用于计数（上报位置） */
        p->num_positions = 0;
        p->detent_kp = 0.0f;
        p->damping = 0.0f;
        break;
    case HAPTIC_DETENT:
        p->width_rad = 2.0f * PI / 12.0f;
        p->num_positions = 0;
        p->detent_kp = 1.1f;
        break;
    case HAPTIC_SWITCH:
        /* 86 盒翘板手感：两位相距 40°，强吸附 */
        p->width_rad = 40.0f * PI / 180.0f;
        p->num_positions = 2;
        p->detent_kp = 1.4f;
        p->snap_point = 0.5f;
        p->dead_zone = 0.03f;
        p->endstop_kp = 2.5f;
        break;
    case HAPTIC_DIMMER:
        /* 270° / 100 格，不做卡位，只给轻阻尼 + 端点 */
        p->width_rad = (270.0f * PI / 180.0f) / 100.0f;
        p->num_positions = 101;
        p->detent_kp = 0.0f;
        p->damping = 0.004f;
        p->dead_zone = 0.0f;
        break;
    case HAPTIC_CURTAIN:
        p->width_rad = 2.0f * PI / 20.0f;
        p->num_positions = 21;
        p->detent_kp = 0.6f;
        break;
    default:
        break;
    }
}

void haptic_init(haptic_state_t *state)
{
    haptic_set_mode(state, HAPTIC_FREE, 0.0f);
}

void haptic_set_mode(haptic_state_t *state, haptic_mode_t mode, float shaft_angle)
{
    if (mode >= HAPTIC_MODE_COUNT) {
        mode = HAPTIC_FREE;
    }
    state->mode = mode;
    params_for_mode(&state->params, mode);
    state->origin = shaft_angle;
    state->position = 0;
    state->value_norm = 0.0f;
}

void haptic_set_detent_count(haptic_state_t *state, int32_t count, float shaft_angle)
{
    if (count < 2) {
        count = 2;
    }
    if (count > 64) {
        count = 64;
    }
    haptic_set_mode(state, HAPTIC_DETENT, shaft_angle);
    state->params.width_rad = 2.0f * PI / (float)count;
    /* 格越窄弹簧越硬：保持 snap 点处峰值电流 ≈0.32A 恒定（12 格时 kp≈1.1 A/rad） */
    const float edge = state->params.width_rad * state->params.snap_point;
    state->params.detent_kp = clampf(0.32f / edge, 0.5f, 4.0f);
}

const char *haptic_mode_name(haptic_mode_t mode)
{
    switch (mode) {
    case HAPTIC_FREE:
        return "free";
    case HAPTIC_DETENT:
        return "detent";
    case HAPTIC_SWITCH:
        return "switch";
    case HAPTIC_DIMMER:
        return "dimmer";
    case HAPTIC_CURTAIN:
        return "curtain";
    default:
        return "unknown";
    }
}

float haptic_compute_current(haptic_state_t *state, float shaft_angle, float shaft_velocity)
{
    const haptic_params_t *p = &state->params;
    const float w = p->width_rad;
    const int32_t n = p->num_positions;

    /* 相对当前格中心的偏差 */
    float err = shaft_angle - (state->origin + (float)state->position * w);

    /* snap：越过阈值换格（有界时不越界）。while 处理快速拧过多格 */
    const float snap = w * p->snap_point;
    while (err > snap && (n == 0 || state->position < n - 1)) {
        state->position++;
        err -= w;
    }
    while (err < -snap && (n == 0 || state->position > 0)) {
        state->position--;
        err += w;
    }

    if (n > 1) {
        state->value_norm = (float)state->position / (float)(n - 1);
    }

    float i_cmd = -p->damping * shaft_velocity;

    const bool out_of_range = (n > 0) &&
        ((state->position == 0 && err < 0.0f) || (state->position == n - 1 && err > 0.0f));

    if (out_of_range && fabsf(err) > w * 0.5f) {
        /* 端点：先走完半格的卡位弹簧，再叠加硬弹簧 */
        const float over = err - copysignf(w * 0.5f, err);
        i_cmd += -p->detent_kp * copysignf(w * 0.5f, err) - p->endstop_kp * over;
    } else if (p->detent_kp > 0.0f) {
        /* 死区：静止在中心附近不出力（消除编码器噪声/摩擦导致的持续电流） */
        const float dz = w * p->dead_zone;
        float e = 0.0f;
        if (err > dz) {
            e = err - dz;
        } else if (err < -dz) {
            e = err + dz;
        }
        i_cmd += -p->detent_kp * e;
    }

    return clampf(i_cmd, -p->i_peak, p->i_peak);
}
