# Smart Knob Panel

ESP32-S3 based haptic Smart Knob Panel designed for 86 wall switch applications.

Use a FOC-controlled BLDC motor as a high-end rotary interface to control lights (on/off + dimming), curtains, mode switching, and other smart home functions. Commands are published via MQTT (and later extensible to other protocols such as Matter).

## Project Goals

1. **Hardware bring-up first**: Get the motor + encoder closed-loop control stable.
2. Implement several practical haptic modes that feel good for daily use.
3. Later integrate with a local smart home server (Homebridge / Home Assistant) via MQTT.
4. Final form factor: 86 panel wall switch. Screen is optional and can be added later.
5. Press detection is **not** planned in the first version (mechanical complexity).

## Hardware

| Component | Model / Spec | Notes |
|-----------|--------------|-------|
| MCU | LILYGO T-Display-S3 (ESP32-S3R8) | 16MB Flash + 8MB PSRAM. Later can be replaced by ESP32-S3 module. |
| Motor | 2804 BLDC | 12N14P, **pole pairs = 7**, 220KV, low cogging, hollow shaft |
| Driver | SimpleFOCmini | 3PWM |
| Encoder | MT6701 | **SPI (SSI)** — not I2C/ABZ for FOC loop |
| Power | 7.4–12V recommended | Motor rated 12V |

See [docs/hardware.md](docs/hardware.md) for pinout and SPI vs I2C vs ABZ rationale.

## Software Stack

- **Framework**: ESP-IDF v5.x
- **FOC Library**: `espressif/esp_simplefoc`
- **Communication**: MQTT (primary, Phase 3)
- **UI**: LVGL (optional, later stage)
- **Task split**: FOC on core 1, console / future MQTT on the other

## Quick start

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Then follow [docs/bringup.md](docs/bringup.md): `ctrl openloop` → `ctrl sensor` → `ctrl angle` → `ctrl haptic` / `mode detent`.

## Haptic Modes (Initial Set)

| Mode | Description | Typical Use |
|------|-------------|-------------|
| **Free / Smooth** | Very low resistance, almost free spinning | Continuous adjustment |
| **Detent** | Discrete positions (e.g. 12 / 24 steps) | Mode selector, multi-level control |
| **Switch** | Strong return to 0° or 180° | Light on/off |
| **Dimmer** | Fine / continuous + mapping to 0–100% | Light dimming |
| **Curtain** | Continuous with endstops | Curtain open/close |

Details: [docs/modes.md](docs/modes.md). Switch modes via serial (`mode …`).

## Pin Assignment (T-Display-S3)

| Function | GPIO | Notes |
|----------|------|-------|
| PWM_A | 1 | SimpleFOCmini |
| PWM_B | 2 | SimpleFOCmini |
| PWM_C | 3 | SimpleFOCmini |
| Encoder CS | 10 | MT6701 CS（SPI_CS） |
| Encoder SCK | 12 | MT6701 CLK（SPI_CLK，注意不是 11） |
| Encoder MISO | 13 | MT6701 DO（SPI_Q） |
| Encoder MOSI | NC | SSI 不接 |

Change pins in [`main/board_config.h`](main/board_config.h) when moving to a bare module.

## Development Plan

1. **Phase 1 – Hardware Bring-up** (current)
   - Open-loop spin / MT6701 angle / closed-loop / haptic Free+Detent
2. **Phase 2 – Haptic Modes** — tune Dimmer / Curtain / strengths
3. **Phase 3 – Communication** — MQTT → Homebridge / Home Assistant
4. **Phase 4 – Productization** — 86 panel, optional LVGL, Matter

## Directory Structure

```
smart-knob-panel/
├── README.md
├── CMakeLists.txt
├── sdkconfig.defaults
├── docs/
│   ├── hardware.md
│   ├── bringup.md
│   └── modes.md
├── main/
│   ├── idf_component.yml      # espressif/esp_simplefoc
│   ├── app_main.cpp
│   ├── board_config.h
│   ├── foc_task.cpp/.h
│   ├── console_cmd.cpp/.h
│   └── haptic/
└── components/                # reserved
```

## References

- Original SmartKnob: https://github.com/scottbez1/smartknob
- Espressif SimpleFOC: https://components.espressif.com/components/espressif/esp_simplefoc
- Espressif FOC knob example: https://github.com/espressif/esp-iot-solution/tree/master/examples/motor/foc_knob_example
- Motor: 2804 BLDC (12N14P, pole pairs = 7)

## Status

Phase 1 scaffold ready: ESP-IDF project builds against `esp_simplefoc`, MT6701 SPI, FOC task + serial console for bring-up and haptic Free/Detent (Switch/Dimmer/Curtain stubbed). MQTT / UI not started.
