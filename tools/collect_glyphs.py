#!/usr/bin/env python3
"""收集 C 源码字符串常量里的非 ASCII 字符（忽略注释），供 lv_font_conv --symbols 使用"""
import re, sys
src = open(sys.argv[1], encoding='utf-8').read()
src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
src = re.sub(r'//[^\n]*', '', src)
chars = set()
for s in re.findall(r'"((?:[^"\\]|\\.)*)"', src):
    chars.update(ch for ch in s if ord(ch) > 0x7F)
chars.update('°·')
print(''.join(sorted(chars)))
