#pragma once

/**
 * @file board_config.h
 * @brief 板级引脚与电机参数 —— 换板子时优先只改本文件。
 *
 * 默认硬件：LILYGO T-Display-S3 + SimpleFOCmini(DRV8313) + MT6701(SPI/SSI) + 2804。
 */

#include "driver/gpio.h"
#include "hal/spi_types.h"  /* SPI2_HOST */
#include "sdkconfig.h"

/* ========== SimpleFOCmini 三相 PWM + EN ==========
 * EN 必须接 MCU（GPIO21），不能常接 3V3。
 * 停机/空闲休眠时软件拉低 EN，半桥 Hi-Z。
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
 * 短按：停 → 自由 → 卡位 → 停
 * 长按：急停
 */
#define BOARD_USER_BTN_GPIO       GPIO_NUM_14

/* ========== 电机：2804 云台电机（实测参数）==========
 * 12N14P → 极对数 7；Y 接，相电阻 2.55Ω（线-线 5.1Ω），估算取保守 2.50Ω
 *   （R 取小 → 同样 Uq 估出的电流偏大 → 限流偏保守）。
 * KV≈220 rpm/V → Ke(线-线峰值) = 1/(220·2π/60) = 0.0434 V/(rad/s)
 * SimpleFOC 的 Uq 为相峰值量纲 → 相反电动势系数 = Ke/√3 ≈ 0.0251 V/(rad/s)
 * Ls=0.86mH：电气时间常数 L/R≈0.34ms，低速旋钮下 ωL 项可忽略 → 不做 d 轴前馈，Ud≡0。
 * 额定 0.5A，极限 2A。
 */
#define BOARD_MOTOR_POLE_PAIRS    7
#define BOARD_MOTOR_RS_OHM        2.50f
#define BOARD_MOTOR_KE_LL         0.0434f
#define BOARD_MOTOR_KE_PHASE      (BOARD_MOTOR_KE_LL / 1.7320508f)
#define BOARD_MOTOR_LS_H          0.00086f
#define BOARD_MOTOR_I_RATED       0.5f
#define BOARD_MOTOR_I_MAX         2.0f
/* 反电动势补偿系数（<1 欠补偿更稳，避免 KV 误差导致过补偿反向推电机） */
#define BOARD_BEMF_COMP           0.9f

/* ========== 供电（Kconfig: Smart Knob Panel → KNOB_SUPPLY_MV）==========
 * 旧版写死 9V 而实物 12V：所有电压指令被放大 12/9=1.33 倍 —— 发热主因之一。
 */
#define BOARD_V_SUPPLY_DEFAULT    (CONFIG_KNOB_SUPPLY_MV / 1000.0f)
#define BOARD_V_SUPPLY_MIN        7.0f
#define BOARD_V_SUPPLY_MAX        12.6f

/* ========== 电流预算（全部为软件估算：I ≈ (Uq − Ke·ω)/Rs）==========
 * 发热 P = 3/2 · I² · Rs（q 轴峰值 I，三相合计）。
 */
#define BOARD_I_LIMIT_DEFAULT     (CONFIG_KNOB_I_LIMIT_MA / 1000.0f)   /* 触感峰值 */
#define BOARD_I_LIMIT_ABS         1.0f    /* 控制台可调上限，绝不超过 */
#define BOARD_I_RMS_LIMIT         (CONFIG_KNOB_I_RMS_LIMIT_MA / 1000.0f)
#define BOARD_I_RMS_TAU_S         4.0f    /* I²t 热模型时间常数 */
#define BOARD_I_OPENLOOP          0.20f   /* 开环诊断：Uq = 0.2A×Rs = 0.5V */
#define BOARD_OPENLOOP_TIMEOUT_MS 10000   /* 开环最长运行，超时自动停 */
/* initFOC 对齐电压（Kconfig），失败按 BOARD_V_ALIGN_STEP 升档重试。
 * 0.5A(1.25V) 实测推不动转子 → 校准失败；改 2.0V 起步（≈0.8A，仅约 2–3s） */
#define BOARD_V_ALIGN             (CONFIG_KNOB_ALIGN_MV / 1000.0f)
#define BOARD_V_ALIGN_MAX         (CONFIG_KNOB_ALIGN_MAX_MV / 1000.0f)
#define BOARD_V_ALIGN_STEP        0.5f

/* ========== 空闲休眠 ==========
 * 静止 + 出力≈0 持续 IDLE_MS → 拉低 EN（驱动冷态）；一动立即唤醒（编码器始终在读）。
 */
#define BOARD_IDLE_VEL            0.30f   /* rad/s */
#define BOARD_IDLE_I              0.02f   /* A */
#define BOARD_IDLE_MS             1500
#define BOARD_WAKE_ANGLE          0.035f  /* 约 2°，休眠中转动超过即唤醒 */

#define BOARD_OPENLOOP_VEL        2.0f

/* ========== FOC 环 ==========
 * 每个 RTOS tick(1ms) 内连续跑 N 次 loopFOC+力矩计算，约 4–6 kHz（受 SSI 读数时间限制），
 * 再 vTaskDelay(1) 让出 CPU（喂 IDLE 看门狗）。旧版每 tick 只跑 1 次 = 1kHz，
 * 7 对极下电角度刷新太慢，转快时电流相位误差大、手感发涩。
 */
#define BOARD_FOC_ITER_PER_TICK   5

/* LEDC 通道（3PWM，20kHz / 9bit，由 esp_simplefoc 决定） */
#define BOARD_LEDC_CH_A           0
#define BOARD_LEDC_CH_B           1
#define BOARD_LEDC_CH_C           2

#define BOARD_FOC_TASK_CORE       1
#define BOARD_FOC_TASK_PRIORITY   5
#define BOARD_FOC_TASK_STACK      8192
