#!/usr/bin/env python3
"""生成旋钮屏预渲染质感资源（LVGL 8 C 数组）→ main/ui/assets/
与 waveshare-amoled-neon-ui 同一视觉语言：香槟金 + 深青、暗角、颗粒、玻璃、内高光。
格式：RGB565（字节交换，配合 CONFIG_LV_COLOR_16_SWAP=y）[+A8]
  img_bg        320×170 不透明背景：暖色暗底 + 左上暖光 + 右下青色微光 + 暗角 + 细颗粒
  img_disc_on   112×112 选中圆盘：玻璃质感 + 香槟金外发光 + 顶部内高光
  img_disc_off  112×112 未选中圆盘：磨砂深灰玻璃
  img_grad_ring 140×140 圆环渐变（金 → 青，锥形），作 lv_arc 指示器贴图
  img_card      156×124 磨砂玻璃卡片（右侧信息区）
  img_hair      300×2   发丝分隔线（两端淡出）
用法：python3 tools/gen_assets.py
"""
import math, os, random

OUT = os.path.join(os.path.dirname(__file__), '..', 'main', 'ui', 'assets')
GOLD = (0xE8, 0xC8, 0x9A)
GOLD_D = (0xB8, 0x8F, 0x55)
TEAL = (0x2B, 0xB3, 0xA6)
random.seed(11)


def clamp(v, lo=0.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def lerp(a, b, t):
    t = clamp(t)
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def add(a, b, k):
    return tuple(a[i] + b[i] * k for i in range(3))


def to565(c):
    r, g, b = (int(clamp(x / 255.0) * 255 + 0.5) for x in c)
    v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    return bytes(((v >> 8) & 0xFF, v & 0xFF))


NAMES = []


def write_img(name, w, h, fn, alpha=True):
    buf = bytearray()
    for y in range(h):
        for x in range(w):
            c, a = fn(x, y)
            buf += to565(c)
            if alpha:
                buf.append(int(clamp(a) * 255 + 0.5))
    cf = 'LV_IMG_CF_TRUE_COLOR_ALPHA' if alpha else 'LV_IMG_CF_TRUE_COLOR'
    with open(os.path.join(OUT, name + '.c'), 'w') as f:
        f.write('/* 由 tools/gen_assets.py 自动生成，请勿手改 */\n#include "lvgl.h"\n')
        f.write('#if LV_COLOR_DEPTH != 16 || LV_COLOR_16_SWAP == 0\n#error "资源按 RGB565 + 字节交换生成"\n#endif\n')
        f.write('static const LV_ATTRIBUTE_LARGE_CONST uint8_t %s_map[] = {\n' % name)
        for i in range(0, len(buf), 32):
            f.write(','.join('0x%02x' % b for b in buf[i:i + 32]) + ',\n')
        f.write('};\nconst lv_img_dsc_t %s = {\n' % name)
        f.write('    .header.cf = %s,\n    .header.always_zero = 0,\n    .header.reserved = 0,\n' % cf)
        f.write('    .header.w = %d,\n    .header.h = %d,\n    .data_size = %d,\n    .data = %s_map,\n};\n'
                % (w, h, len(buf), name))
    NAMES.append(name)


def smooth(e0, e1, x):
    t = clamp((x - e0) / (e1 - e0))
    return t * t * (3 - 2 * t)


# ---------- 背景 ----------
def bg(x, y):
    W, H = 320.0, 170.0
    base = (9, 9, 10)
    c = base
    # 左上暖光
    d = math.hypot((x - 40) / W, (y + 20) / H * 0.8)
    c = add(c, (60, 44, 24), clamp(1 - d / 0.75) ** 2.2)
    # 右下青色微光
    d2 = math.hypot((x - 300) / W, (y - 190) / H * 0.8)
    c = add(c, (8, 34, 32), clamp(1 - d2 / 0.6) ** 2.4)
    # 暗角
    vx, vy = (x / W - 0.5) * 2, (y / H - 0.5) * 2
    v = clamp(1 - 0.38 * (vx * vx * 0.6 + vy * vy))
    c = tuple(ch * v for ch in c)
    # 颗粒
    n = (random.random() - 0.5) * 5
    c = tuple(ch + n for ch in c)
    return c, 1


# ---------- 圆盘 ----------
def disc(on):
    S = 112
    R = 40.0
    cx = cy = S / 2 - 0.5

    def f(x, y):
        dx, dy = x - cx, y - cy
        r = math.hypot(dx, dy)
        if r <= R + 0.5:
            t = (dy / R + 1) / 2  # 0 顶 1 底
            if on:
                top, bot = (64, 54, 40), (22, 19, 15)
            else:
                top, bot = (40, 40, 42), (16, 16, 18)
            c = lerp(top, bot, t)
            # 拉丝纹理（同心圆）
            c = add(c, (255, 255, 255), 0.018 * math.sin(r * 2.3))
            # 顶部内高光
            hl = clamp(1 - math.hypot(dx / R, (dy + R * 0.55) / (R * 0.6)))
            c = add(c, (255, 245, 225), 0.22 * hl ** 1.6)
            # 边缘描边
            edge = smooth(R - 2.2, R - 0.3, r)
            ec = GOLD if on else (90, 88, 84)
            c = lerp(c, ec, edge * (0.85 if on else 0.45) * (1 - t * 0.5))
            a = clamp(R + 0.5 - r)
            # 外缘抗锯齿与发光叠加
            if a < 1 and on:
                g = clamp(1 - (r - R) / 14)
                return lerp(GOLD, c, a), max(a, 0.5 * g ** 2)
            return c, a
        # 外发光 / 投影
        if on:
            g = clamp(1 - (r - R) / 15) ** 2.2
            return GOLD, 0.55 * g
        g = clamp(1 - (r - R) / 10) ** 2
        return (0, 0, 0), 0.6 * g
    return f


# ---------- 环形渐变 ----------
def grad_ring(x, y):
    S = 140
    cx = cy = S / 2 - 0.5
    ang = math.degrees(math.atan2(y - cy, x - cx))  # -180..180, 0 = 右
    # arc 起点 135°（左下），顺时针 270°
    t = ((ang - 135) % 360) / 270.0
    c = lerp(GOLD_D, GOLD, t * 1.6) if t < 0.6 else lerp(GOLD, TEAL, (t - 0.6) / 0.4)
    return c, 1


# ---------- 玻璃卡片 ----------
def card(x, y):
    W, H, RR = 156, 124, 16

    def sd(px, py):
        qx = abs(px - W / 2 + 0.5) - (W / 2 - RR)
        qy = abs(py - H / 2 + 0.5) - (H / 2 - RR)
        return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - RR
    d = sd(x, y)
    if d > 0.5:
        return (0, 0, 0), 0
    a = clamp(0.5 - d)
    t = y / H
    c = lerp((34, 32, 30), (18, 18, 19), t)
    c = add(c, (255, 240, 215), 0.05 * clamp(1 - t * 3))
    n = (random.random() - 0.5) * 4
    c = tuple(ch + n for ch in c)
    # 描边：顶部亮，底部暗
    edge = smooth(-1.6, -0.2, d)
    c = lerp(c, lerp((120, 110, 95), (40, 40, 40), t), edge * 0.8)
    return c, a * 0.86


def hair(x, y):
    t = x / 299.0
    a = math.sin(math.pi * t) ** 1.5
    return lerp(GOLD, TEAL, t), a * (0.5 if y == 0 else 0.12)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    write_img('img_bg', 320, 170, bg, alpha=False)
    write_img('img_disc_on', 112, 112, disc(True))
    write_img('img_disc_off', 112, 112, disc(False))
    write_img('img_grad_ring', 140, 140, grad_ring, alpha=False)
    write_img('img_card', 156, 124, card)
    write_img('img_hair', 300, 2, hair)
    with open(os.path.join(OUT, 'ui_assets.h'), 'w') as f:
        f.write('#pragma once\n/* 由 tools/gen_assets.py 自动生成 */\n#include "lvgl.h"\n')
        for n in NAMES:
            f.write('LV_IMG_DECLARE(%s);\n' % n)
    print('done:', ', '.join(NAMES))
