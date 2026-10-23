# Haptic Modes

Parameters are runtime-adjustable where noted. Implementation: [`main/haptic/haptic_mode.c`](../main/haptic/haptic_mode.c).

## 1. Free / Smooth (`mode free`)

- Near-zero torque + light damping
- Continuous fine adjustment

## 2. Detent (`mode detent`)

- Fixed positions per revolution (default **12**, change with `detent N`)
- Strength / snap defaults in firmware; tune in `haptic_set_mode`
- Typical use: mode selector, multi-level scenes

## 3. Switch (`mode switch`)

- Strong restoring torque toward **0° or 180°**
- Feels like a mechanical toggle
- Typical use: light on/off

## 4. Dimmer (`mode dimmer`) — stub feel

- Soft endstops over a limited travel; `value_norm` maps toward 0–100%
- Strengths are placeholders — refine after Free/Detent are good
- Typical use: brightness, volume

## 5. Curtain (`mode curtain`) — stub feel

- Continuous travel with stronger endstops
- Typical use: curtain / blind open–close

## Control path

Haptic modes run under `ctrl haptic` (torque mode). Bring-up paths:

- `ctrl openloop` — no encoder closed loop
- `ctrl sensor` — encoder only
- `ctrl angle` — position hold
- `ctrl haptic` — torque from haptic engine

Mode change: serial console (`mode …`). BOOT button switching can be added later.
