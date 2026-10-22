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
| Encoder | MT6701 | SPI interface |
| Power | 7.4–12V recommended | Motor rated 12V |

## Software Stack

- **Framework**: ESP-IDF v5.x
- **FOC Library**: `espressif/esp_simplefoc` (recommended for pure ESP-IDF)
- **Communication**: MQTT (primary)
- **UI**: LVGL (optional, later stage)
- **Task split**: FOC control on one core, communication / UI on the other

## Haptic Modes (Initial Set)

These are the modes to implement first. Parameters (detent count, strength, snap point, etc.) should be configurable.

| Mode | Description | Typical Use |
|------|-------------|-------------|
| **Free / Smooth** | Very low resistance, almost free spinning | Continuous adjustment |
| **Detent** | Discrete positions (e.g. 12 / 24 steps) | Mode selector, multi-level control |
| **Switch** | Strong return to 0° or 180° | Light on/off |
| **Dimmer** | Fine detents or continuous + mapping to 0–100% | Light dimming |
| **Curtain** | Continuous or coarse detents with endstops | Curtain open/close |

Mode switching can initially be done via BOOT button or serial command.

## Pin Assignment (T-Display-S3)

LCD parallel bus occupies many pins. Recommended free pins:

| Function | GPIO | Notes |
|----------|------|-------|
| PWM_A | 1 | SimpleFOCmini |
| PWM_B | 2 | SimpleFOCmini |
| PWM_C | 3 | SimpleFOCmini |
| Encoder CS | 10 | MT6701 SPI |
| Encoder SCK | 11 | MT6701 SPI |
| Encoder MOSI | 12 | MT6701 SPI |
| Encoder MISO | 13 | MT6701 SPI |
| Enable / Fault | 14 or 21 | Optional |

When migrating to a bare ESP32-S3 module, only the pin definitions need to be updated.

## Development Plan

1. **Phase 1 – Hardware Bring-up**
   - Open-loop spin
   - Read MT6701 angle reliably
   - Closed-loop position control

2. **Phase 2 – Haptic Modes**
   - Implement the modes listed above
   - Tune strength / snap / endstop feel

3. **Phase 3 – Communication**
   - MQTT publish of current mode + value
   - Basic integration with Homebridge or Home Assistant

4. **Phase 4 – Productization**
   - 86 panel mechanical design
   - Optional screen + LVGL
   - Matter support (optional)

## Directory Structure (Planned)

```
smart-knob-panel/
├── README.md
├── docs/                 # Design notes, pinouts, mode descriptions, etc.
├── main/                 # ESP-IDF main component
├── components/           # Custom components (if any)
├── idf_component.yml     # Dependency on espressif/esp_simplefoc
└── CMakeLists.txt
```

## References

- Original SmartKnob: https://github.com/scottbez1/smartknob
- Espressif SimpleFOC: https://components.espressif.com/components/espressif/esp_simplefoc
- Motor: 2804 BLDC (12N14P, pole pairs = 7)

## Status

Early stage — hardware bring-up and basic FOC closed-loop in progress.
