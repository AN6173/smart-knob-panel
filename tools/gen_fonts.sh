#!/usr/bin/env bash
# 生成旋钮屏 UI 字体（lv_font_conv 4bpp，只含 UI 实际用到的字形）
# 依赖与 waveshare-amoled-neon-ui 相同：node + lv_font_conv@1.5.2、fontTools、
#   Inter / Noto Sans SC（OFL-1.1）、Tabler Icons（MIT）
# 修改 main/ui/ui_knob.c 里的中文文案或图标后需重新运行本脚本。
set -e
cd "$(dirname "$0")/.."
W=${FONT_WORK:-$HOME/.local/fonttools-work}
NODE=${NODE:-node}
CONV="$NODE $W/node_modules/lv_font_conv/lv_font_conv.js"
INTER=${INTER:-/usr/share/fonts/opentype/inter}
TABLER=${TABLER:-$W/node_modules/@tabler/icons-webfont/dist/fonts/tabler-icons-outline.ttf}
OUT=main/ui/fonts
CJK=$(python3 tools/collect_glyphs.py main/ui/ui_knob.c)
echo "中文字形: $CJK"
COMMON="--bpp 4 --format lvgl --no-compress --lv-include lvgl.h"
LAT="-r 0x20-0x7E,0xB0,0xB7,0x2013"
# 与 ui_knob.c 中 IC_* 同步
ICONS_MODE=0xEF85,0xEFB1,0xEB3F,0xEA51,0xEF06
ICONS_STAT=0xEA38,0xEAF8,0xEA06,0xEB38,0xED4A,0xED23
SUB=$W/tabler-knob-subset.ttf
${PY:-python3} -m fontTools.subset "$TABLER" --unicodes="$(echo $ICONS_MODE,$ICONS_STAT | sed 's/0x/U+/g')" \
    --layout-features= --drop-tables+=GSUB,GPOS,GDEF --output-file=$SUB
# 16px 正文字体内嵌状态栏图标（同一 label 里图标 + 文字）
$CONV $COMMON --size 16 --font $INTER/Inter-Regular.otf $LAT --font $W/NotoSansSC-DemiLight.otf --symbols "$CJK" \
    --font $SUB -r $ICONS_STAT -o $OUT/font_cn_16.c
$CONV $COMMON --size 20 --font $INTER/Inter-Light.otf $LAT --font $W/NotoSansSC-Light.otf --symbols "$CJK" -o $OUT/font_cn_20.c
$CONV $COMMON --size 28 --font $INTER/Inter-Light.otf $LAT --font $W/NotoSansSC-Light.otf --symbols "$CJK" -o $OUT/font_cn_28.c
$CONV $COMMON --size 52 --font $INTER/Inter-ExtraLight.otf --symbols "0123456789.%-+°" -o $OUT/font_num_52.c
$CONV $COMMON --size 40 --font $SUB -r $ICONS_MODE -o $OUT/font_icon_40.c
$CONV $COMMON --size 16 --font $SUB -r $ICONS_MODE,$ICONS_STAT -o $OUT/font_icon_16.c
echo done
