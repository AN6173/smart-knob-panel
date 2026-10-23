/**
 * @file haptic_mode.c
 * @brief 触感力矩计算（参考 SmartKnob / esp foc_knob 的虚拟弹簧思路，简化版）。
 */

#include "haptic_mode.h"

#include <math.h>

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

/** 角度归一化到 (-π, π] */
static float wrap_pm_pi(float a)
{
    while (a > PI) {
        a -= 2.0f * PI;
    }
    while (a < -PI) {
        a += 2.0f * PI;
    }
    return a;
}

void haptic_init(haptic_state_t *state)
{
    state->mode = HAPTIC_FREE;
    state->position = 0;
    state->value_norm = 0.0f;
    state->params.detent_count = 12;
    state->params.detent_strength = 1.0f;
    state->params.snap_point = 1.1f;
    state->params.endstop_strength = 1.5f;
    state->params.range_rad = 2.0f * PI;
}

void haptic_set_mode(haptic_state_t *state, haptic_mode_t mode)
{
    if (mode >= HAPTIC_MODE_COUNT) {
        mode = HAPTIC_FREE;
    }
    state->mode = mode;
    state->position = 0;
    state->value_norm = 0.0f;

    switch (mode) {
    case HAPTIC_FREE:
        state->params.detent_count = 0;
        state->params.detent_strength = 0.05f;
        break;
    case HAPTIC_DETENT:
        state->params.detent_count = 12;
        state->params.detent_strength = 0.35f;
        state->params.snap_point = 1.1f;
        break;
    case HAPTIC_SWITCH:
        state->params.detent_count = 2;
        state->params.detent_strength = 1.5f;
        state->params.snap_point = 0.55f;
        state->params.range_rad = PI; /* 0° ↔ 180° */
        break;
    case HAPTIC_DIMMER:
        state->params.detent_count = 0;
        state->params.detent_strength = 0.2f;
        state->params.endstop_strength = 1.5f;
        state->params.range_rad = 270.0f * PI / 180.0f;
        break;
    case HAPTIC_CURTAIN:
        state->params.detent_count = 0;
        state->params.detent_strength = 0.3f;
        state->params.endstop_strength = 2.0f;
        state->params.range_rad = 2.0f * PI;
        break;
    default:
        break;
    }
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

/** 把角度误差拉向 0 的比例力矩，并限幅 */
static float torque_toward(float error, float strength, float limit)
{
    float t = -strength * error;
    return clampf(t, -limit, limit);
}

float haptic_compute_torque(haptic_state_t *state, float shaft_angle, float shaft_velocity)
{
    const float torque_limit = 0.8f; /* 电压模式≈伏特；上限低 → 持续运行也不易烫 */
    /* 自由模式零阻尼；卡位只在偏离齿槽时出力，静止时指令≈0 */
    const float damping = (state->mode == HAPTIC_FREE) ? 0.0f : 0.008f;
    float torque = -damping * shaft_velocity;

    switch (state->mode) {
    case HAPTIC_FREE:
        break;

    case HAPTIC_DETENT: {
        const int32_t n = state->params.detent_count > 0 ? state->params.detent_count : 12;
        const float width = 2.0f * PI / (float)n;
        const float half = width * 0.5f;

        /* 映射到当前格子局部坐标，中心在 half */
        float local = fmodf(shaft_angle, width);
        if (local < 0.0f) {
            local += width;
        }
        float to_center = local - half;
        if (fabsf(to_center) > half * state->params.snap_point) {
            /* 越过 snap 点 → 计入下一格 */
            int32_t step = (to_center > 0.0f) ? 1 : -1;
            state->position += step;
            to_center -= (float)step * width;
        }
        torque += torque_toward(to_center, state->params.detent_strength, torque_limit);
        break;
    }

    case HAPTIC_SWITCH: {
        /* 双稳态：吸附到 0 或 π 中更近的一个 */
        const float a0 = 0.0f;
        const float a1 = PI;
        float e0 = wrap_pm_pi(shaft_angle - a0);
        float e1 = wrap_pm_pi(shaft_angle - a1);
        float err = (fabsf(e0) < fabsf(e1)) ? e0 : e1;
        state->position = (fabsf(e0) < fabsf(e1)) ? 0 : 1;
        state->value_norm = (float)state->position;
        torque += torque_toward(err, state->params.detent_strength * 1.5f, torque_limit);
        break;
    }

    case HAPTIC_DIMMER:
    case HAPTIC_CURTAIN: {
        /* stub：软端点 + value_norm，手感参数上板后再调 */
        const float lo = 0.0f;
        const float hi = state->params.range_rad;
        float a = shaft_angle;
        while (a < lo) {
            a += hi;
        }
        while (a > hi) {
            a -= hi;
        }
        state->value_norm = clampf(a / hi, 0.0f, 1.0f);

        if (shaft_angle < lo) {
            torque += torque_toward(shaft_angle - lo, state->params.endstop_strength, torque_limit);
        } else if (shaft_angle > hi) {
            torque += torque_toward(shaft_angle - hi, state->params.endstop_strength, torque_limit);
        } else {
            torque += -state->params.detent_strength * 0.1f * shaft_velocity;
        }
        break;
    }

    default:
        break;
    }

    return clampf(torque, -torque_limit, torque_limit);
}
