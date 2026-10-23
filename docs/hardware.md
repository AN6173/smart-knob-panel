# Hardware

## Overview

ESP32-S3 (LILYGO T-Display-S3) drives a 2804 BLDC through SimpleFOCmini (3PWM). Shaft angle comes from an MT6701 magnetic encoder over **SPI (SSI)**.

## Bill of materials

| Part | Spec | Notes |
|------|------|-------|
| MCU | T-Display-S3 (ESP32-S3R8) | 16MB Flash + 8MB PSRAM |
| Motor | 2804 BLDC | 12N14P, **pole pairs = 7**, ~220KV |
| Driver | SimpleFOCmini | 3PWM |
| Encoder | MT6701 | SPI/SSI primary |
| Supply | 7.4–12V | Motor rated 12V; start with limited voltage in firmware |

## Pinout / 接线定义 (T-Display-S3)

源文件：[`main/board_config.h`](../main/board_config.h)

### MCU ↔ SimpleFOCmini（电机驱动）

| MCU (ESP32-S3) | SimpleFOCmini | 说明 |
|----------------|---------------|------|
| GPIO 1 | IN1 / PWM_A | 相 A |
| GPIO 2 | IN2 / PWM_B | 相 B |
| GPIO 3 | IN3 / PWM_C | 相 C |
| GPIO 21 | **EN** | 使能；**必须接 MCU，禁止常接 3V3**（停机拉低才不烫） |
| GPIO 14 | 用户按键→GND | 短按切模式 / 长按急停 |
| GND | GND | 共地（必须） |
| — | VM / VIN | **电机电源 7.4–12V**（不要从 USB 给电机供电） |

SimpleFOCmini 三相输出 → 2804 的 U/V/W。若转向反了，任换两根电机线。

### MCU ↔ MT6701（磁编码器 SPI/SSI）

按 **T-Display-S3 排针丝印** 接（与板载 FSPI 一致）：

| MCU 排针丝印 | GPIO | MT6701 模块 | 说明 |
|--------------|------|-------------|------|
| SPI_CS | **10** | CS / nCS | 片选，**必须接** |
| SPI_CLK | **12** | CLK | 时钟（**不要接到 11**） |
| SPI_Q | **13** | DO / MISO | 数据出 |
| SPI_D | 11 | — | 本工程不用（SSI 无 MOSI） |
| 3V3 | — | VCC | 3.3V |
| GND | — | GND | 共地 |

串口诊断（`pos ... raw=... crc=...`）：

| raw / crc | 含义 | 先查 |
|-----------|------|------|
| `raw=0xFFFFFF` + `BUS_OPEN` | MISO 一直被读成 1，芯片没在驱动 DO | 编码器 3V3/GND；DO→SPI_Q(13)；CS→SPI_CS(10)；模块是否真有 **CSN** 脚 |
| `crc=BAD` 且 raw 会变 | 有数据但相位/时序不对 | CLK 是否在 **GPIO12**；MODE 是否为 SSI（勿焊成纯 ABZ） |
| `crc=OK` 且角度随转 | 正常 | — |

常见翻车：
1. CLK 接到 GPIO11（丝印 SPI_D=MOSI）→ 必须接 **SPI_CLK=GPIO12**
2. 模块只有 A/B/Z 或只有 SDA/SCL、**没有 CS** → 不能走本工程的 SSI
3. 模块 MODE/焊盘选成 ABZ → DO 不输出 SSI
4. 磁环气隙过大 → raw 乱跳或 CRC BAD

本仓库**没有单独的整机 PDF 原理图**；MCU 侧以 [LILYGO T-Display-S3 官方原理图](https://github.com/Xinyuan-LilyGO/T-Display-S3/blob/main/schematic/T_Display_S3.pdf) + [`main/board_config.h`](../main/board_config.h) 为准。

### 电源拓扑（推荐）

```
12V 电池/电源 ──┬── SimpleFOCmini VM ── 2804
                │
                └──（可选降压）── 不建议直接给 T-Display 灌 12V
USB 5V ──────────── T-Display-S3（MCU + 编码器 3V3）
GND 全系统共地
```

LCD 并口引脚留给后续 LVGL，本阶段不要占用。

## Encoder interface: SPI vs I2C vs ABZ

| Interface | Use as FOC feedback? | Notes |
|-----------|----------------------|-------|
| **SPI / SSI** | **Yes — default** | 14-bit absolute angle, fast, no index search. `esp_simplefoc` supports it natively. |
| I2C | No (config only) | Same 14-bit data but too slow / bus-blocking for the FOC loop. EEPROM write often needs ~5V. |
| ABZ | Optional later | Low latency incremental; needs Z index; PPR must be programmed. Not for first bring-up. |

**Requirement:** the MT6701 module must expose **CS**. Without CS, SSI/SPI mode cannot be used.

## Power & safety

- Common GND between MCU, driver, and encoder.
- Keep magnet air gap small and concentric; large gap breaks `initFOC` pole-pair alignment.
- First power-up: firmware voltage limit defaults to **6V** (`BOARD_V_LIMIT`) even if supply is 12V. Raise after closed-loop is stable.
- Do not stall the motor at high voltage for long periods.

## Mechanical (later)

86-panel enclosure and optional screen are out of scope for Phase 1. Press detection is not planned for the first hardware revision.
