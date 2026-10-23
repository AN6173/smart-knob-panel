#!/usr/bin/env bash
# 可靠串口监视（绕过 Cursor AppImage 对 sys.executable 的破坏）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="/home/hao/.local/esp-python-shim:/home/hao/.local/bin:${PATH:-}"
export IDF_PATH="${IDF_PATH:-/home/hao/esp/esp-idf}"
# shellcheck disable=SC1091
. "$IDF_PATH/export.sh"
PORT="${1:-/dev/ttyACM0}"
cd "$ROOT"
exec idf.py -p "$PORT" monitor
