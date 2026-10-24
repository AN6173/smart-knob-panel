# Hardware Bring-up

## Prerequisites

- ESP-IDF v5.4.x（本机路径：`~/esp/esp-idf`，工具：`~/.espressif`）
- 激活环境：`get_idf`（已写入 `~/.zshrc`）或 `. ~/esp/export-idf.sh`
- Hardware wired per [hardware.md](hardware.md)
- USB cable to T-Display-S3

## Build & flash

```bash
get_idf
cd ~/WorkSpace/smart-knob-panel
idf.py set-target esp32s3   # 仅首次或换芯片时需要
idf.py build
idf.py -p /dev/ttyACM0 flash monitor   # 按实际串口改端口
```

固件产物：

| 文件 | 说明 |
|------|------|
| `build/smart-knob-panel.bin` | 应用固件 |
| `build/bootloader/bootloader.bin` | Bootloader |
| `build/partition_table/partition-table.bin` | 分区表 |

On success you should see `motor init OK` and a `knob>` console prompt.

## Recommended sequence（编码器正常后）

监视器里先 `stream off`，再：

```
test spin          # 开环慢转约 4s，自动 stop
test hold          # 闭环锁当前角，用手拧应回弹
test step 0.5      # 步进 0.5 rad
test detent 12     # 虚拟卡位
stop               # 随时急停
```

手动细调也可用：`ctrl openloop` / `ctrl angle` / `mode detent`、`ilim 0.2`（先小电流）、`target ...`。

## Console commands

| Command | Description |
|---------|-------------|
| `help` / `status` / `stream on\|off` | 基础 |
| `stop` | 急停 → sensor |
| `ilim <A>` | 峰值电流上限（默认 0.35A） |
| `vlim <V>` | Uq 硬上限 |
| `vsup <V>` | 供电电压（须与实物一致） |
| `power` | 估算电流 / 铜损 / 休眠 |
| `recal` | 清除电角度校准并重启 |
| `test spin\|hold\|detent\|step` | 快捷电机测试 |
| `ctrl sensor\|openloop\|angle\|haptic` | 控制路径 |
| `mode free\|detent\|switch\|dimmer\|curtain` | 触感 |
| `target <float>` | 开环速度或闭环角 |
| `detent <2..64>` | 卡位数量 |

## Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| `initFOC` wrong pole pairs | Magnet gap / eccentricity; confirm pp=7 |
| Angle frozen | CS/SCK/MISO wiring; CS must be connected |
| Motor buzzes, no torque | Phase order; try swapping two phases |
| Overheat / high current | Lower `BOARD_V_LIMIT`; check shorted phases |
| Console garbled | Baud 115200; only one process on the serial port |

## Next steps

After Free + Detent feel acceptable, tune strengths in `haptic/haptic_mode.c`, then move to MQTT (Phase 3).
