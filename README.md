# Smart Knob Panel

ESP32-S3 based haptic Smart Knob Panel for 86 wall switch applications.

Use FOC-controlled BLDC motor as a high-end rotary interface to control lights (on/off + dimming), curtains, mode switching, and more. Commands are sent out via MQTT or other protocols.

## Hardware

- **MCU**: LILYGO T-Display-S3 (ESP32-S3R8, 16MB Flash + 8MB PSRAM)
- **Motor**: 2804 BLDC (12N14P, pole pairs = 7, low cogging, hollow shaft)
- **Driver**: SimpleFOCmini (3PWM)
- **Encoder**: MT6701 magnetic encoder (SPI)
- **Target form factor**: 86 panel wall switch

> Press detection (HX711 / strain gauge) is intentionally omitted for mechanical simplicity.

## Software Stack

- **Framework**: ESP-IDF v5.x
- **FOC Library**: `espressif/esp_simplefoc` (recommended)
- **UI** (optional, later stage): LVGL
- **Communication**: MQTT (primary), extensible to other protocols

## Features (Planned)

- Multiple haptic profiles:
  - Binary switch (light on/off)
  - Detent-based dimming / volume
  - Continuous curtain control
  - Mode selector with strong endstops
- MQTT publish of state changes
- Configurable detent strength, snap points, and endstops
- Dual-core task split (FOC on one core, communication/UI on the other)

## Pin Assignment (T-Display-S3, non-IMU version)

| Function       | GPIO  | Notes                  |
|----------------|-------|------------------------|
| PWM_A          | 1     | SimpleFOCmini          |
| PWM_B          | 2     | SimpleFOCmini          |
| PWM_C          | 3     | SimpleFOCmini          |
| Encoder CS     | 10    | MT6701 SPI             |
| Encoder SCK    | 11    | MT6701 SPI             |
| Encoder MOSI   | 12    | MT6701 SPI             |
| Encoder MISO   | 13    | MT6701 SPI             |
| Enable / Fault | 14/21 | Optional               |

LCD parallel bus occupies many pins; free GPIOs are mainly on the left header.

## References

- Original SmartKnob: https://github.com/scottbez1/smartknob
- Espressif SimpleFOC component: https://components.espressif.com/components/espressif/esp_simplefoc
- Motor datasheet: 2804 BLDC (12N14P, 220KV)

## Status

Early stage — hardware bring-up and basic FOC closed-loop in progress.
