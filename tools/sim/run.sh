#!/usr/bin/env bash
# PC 预览：编译 LVGL 8.3.11 + ui_knob.c，渲染画面到 tools/sim/out/*.png
# 依赖：gcc、python3 + Pillow；LVGL 源码路径 LVGL_DIR（默认 managed_components/lvgl__lvgl）
set -e
cd "$(dirname "$0")"
ROOT=../..
LVGL_DIR=${LVGL_DIR:-$ROOT/managed_components/lvgl__lvgl}
mkdir -p out build
SRCS="$(find $LVGL_DIR/src -name '*.c') $ROOT/main/ui/ui_knob.c $(ls $ROOT/main/ui/fonts/*.c $ROOT/main/ui/assets/*.c) sim_main.c"
gcc -O1 -w -DUI_SIM -DLV_CONF_INCLUDE_SIMPLE -I. -I$LVGL_DIR -I$LVGL_DIR/.. -I$ROOT/main $SRCS -lm -o build/sim
./build/sim
python3 - <<'PY'
import glob, os
from PIL import Image
for f in sorted(glob.glob('out/*.raw')):
    d = open(f, 'rb').read(); im = Image.new('RGB', (320, 170)); px = im.load()
    for i in range(320 * 170):
        v = (d[2*i] << 8) | d[2*i+1]          # LV_COLOR_16_SWAP：高字节在前
        px[i % 320, i // 320] = (((v >> 11) & 31) * 255 // 31, ((v >> 5) & 63) * 255 // 63, (v & 31) * 255 // 31)
    im.resize((640, 340), Image.LANCZOS).save(f[:-4] + '.png'); os.remove(f)
print('png ok')
PY
