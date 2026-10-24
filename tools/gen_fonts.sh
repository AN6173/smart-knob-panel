#!/usr/bin/env bash
# 生成旋钮屏 UI 字体（lv_font_conv 4bpp，只含 UI 实际用到的字形）
# 依赖与 waveshare-amoled-neon-ui 相同：node + lv_font_conv@1.5.2，Inter + Noto Sans SC（均 OFL-1.1）
# 修改 main/ui/ui_knob.c 里的中文文案后需重新运行本脚本。
set -e
cd "$(dirname "$0")/.."
W=${FONT_WORK:-$HOME/.local/fonttools-work}
NODE=${NODE:-$HOME/.local/node/bin/node}
CONV="$NODE $W/node_modules/lv_font_conv/lv_font_conv.js"
INTER=/usr/share/fonts/opentype/inter
OUT=main/ui/fonts
CJK=$(python3 tools/collect_glyphs.py main/ui/ui_knob.c)
echo "中文字形: $CJK"
COMMON="--bpp 4 --format lvgl --no-compress --lv-include lvgl.h"
LAT="-r 0x20-0x7E,0xB0,0xB7,0x2013,0x2248"
$CONV $COMMON --size 16 --font $INTER/Inter-Regular.otf $LAT --font $W/NotoSansSC-DemiLight.otf --symbols "$CJK" -o $OUT/font_cn_16.c
$CONV $COMMON --size 22 --font $INTER/Inter-Light.otf $LAT --font $W/NotoSansSC-Light.otf --symbols "$CJK" -o $OUT/font_cn_22.c
$CONV $COMMON --size 36 --font $INTER/Inter-Light.otf $LAT --font $W/NotoSansSC-Light.otf --symbols "$CJK" -o $OUT/font_cn_36.c
$CONV $COMMON --size 48 --font $INTER/Inter-ExtraLight.otf --symbols "0123456789.%-+°" -o $OUT/font_num_48.c
echo done
