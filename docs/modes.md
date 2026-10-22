# Haptic Modes

Initial set of modes to implement. All parameters should be runtime-configurable where possible.

## 1. Free / Smooth
- Near-zero torque
- Used for continuous fine adjustment

## 2. Detent
- Fixed number of positions (configurable, e.g. 8 / 12 / 24)
- Adjustable detent strength and snap point
- Typical use: mode selector, multi-level scenes

## 3. Switch
- Strong restoring torque toward 0° or 180°
- Feels like a mechanical toggle
- Typical use: light on/off

## 4. Dimmer
- Either fine detents or smooth with soft endstops
- Angle mapped to 0–100%
- Typical use: light brightness, volume

## 5. Curtain
- Continuous or coarse detents
- Hard or soft endstops at fully open / fully closed
- Typical use: curtain / blind control

## Implementation Notes

- Reference the detent / endstop math from the original SmartKnob firmware.
- Use torque mode of `esp_simplefoc`.
- Mode change can initially be triggered by BOOT button or serial command.
