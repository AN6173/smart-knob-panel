#pragma once

/**
 * @file board_config.h
 * @brief 板级引脚与电机参数 —— 换板子时优先只改本文件。
 *
 * 默认硬件：LILYGO T-Display-S3 + SimpleFOCmini + MT6701(SPI/SSI) + 2804。
 */

#include "driver/gpio.h"
#include "hal/spi_types.h"  /* SPI2_HOST */

/* ========== SimpleFOCmini 三相 PWM + EN ==========
 * EN 必须接 MCU（GPIO21），不能常接 3V3。
 * 停机时软件拉低 EN，半桥 Hi-Z，否则必烫。
 */
#define BOARD_PWM_A_GPIO          GPIO_NUM_1
#define BOARD_PWM_B_GPIO          GPIO_NUM_2
#define BOARD_PWM_C_GPIO          GPIO_NUM_3
#define BOARD_DRIVER_ENABLE_GPIO  GPIO_NUM_21

/* ========== MT6701 SPI(SSI) ==========
 * 按 T-Display-S3 排针丝印 FSPI 对齐（勿接错 CLK！）：
 *   CS  → GPIO10 (SPI_CS)
 *   CLK → GPIO12 (SPI_CLK)   ← 注意不是 11
 *   DO/MISO → GPIO13 (SPI_Q)
 *   MOSI 不接
 * 模块必须引出 CS，否则无法 SSI。
 */
#define BOARD_ENC_SPI_HOST        SPI2_HOST
#define BOARD_ENC_SCK_GPIO        GPIO_NUM_12
#define BOARD_ENC_MISO_GPIO       GPIO_NUM_13
#define BOARD_ENC_MOSI_GPIO       ((gpio_num_t)-1)
#define BOARD_ENC_CS_GPIO         GPIO_NUM_10

/* ========== 用户按键（对地按下，内部上拉）==========
 * 短按：停 → 自由(可一直跑) → 卡位(可一直跑) → 停
 * 长按：急停
 */
#define BOARD_USER_BTN_GPIO       GPIO_NUM_14

/* ========== 电机 ==========
 * 2804：12N14P → 极对数 = 7
 */
#define BOARD_MOTOR_POLE_PAIRS    7

/*
 * 长期运行策略（不要用开环一直转）：
 *   开环 Uq≡Vlim → 电流一直有 → 必烫。换库也改变不了物理。
 *   智能旋钮：闭环力矩，空闲指令≈0，只有拧动/卡位时才出力 → 可一直使能且不烫。
 */
#define BOARD_V_SUPPLY            9.0f
#define BOARD_V_LIMIT             2.5f   /* 触感电压上限 */
#define BOARD_V_OPENLOOP          1.0f   /* 仅诊断用，勿长时间开环 */
#define BOARD_V_ALIGN             3.5f   /* initFOC 校准（仅上电数秒，可高于 V_LIMIT） */
#define BOARD_OPENLOOP_VEL        2.0f
#define BOARD_OPENLOOP_SECONDS    2

/* LEDC 通道（3PWM） */
#define BOARD_LEDC_CH_A           0
#define BOARD_LEDC_CH_B           1
#define BOARD_LEDC_CH_C           2

#define BOARD_FOC_TASK_CORE       1
#define BOARD_FOC_TASK_PRIORITY   5
#define BOARD_FOC_TASK_STACK      8192
