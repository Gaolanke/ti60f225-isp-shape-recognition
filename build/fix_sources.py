#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fix_sources.py -- 保证 Efinity 工程文件列表里的形状识别模块是 shape_ink.

背景: Efinity 在编译结束时会按它自己记的状态重写 project XML, 会把已经
不再使用的 rtl/edge/shape_rec.v 塞回文件列表. 那个文件已改名为 .bak, 于是
下一次编译会在 elaborate 阶段报 "cannot open Verilog file".

这个脚本在每次编译前跑一遍, 把列表里的 shape_rec 换成 shape_ink;
要是两个都不在, 就补一条. 幂等, 重复执行无副作用.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(ROOT, "ti60f225_oob.xml")

OLD = '<efx:design_file name="rtl/edge/shape_rec.v" version="default" library="default"/>'
OLD_ALT = '<efx:design_file name="rtl/edge/shape_rec.v" version="default" library="default" />'
NEW = '<efx:design_file name="rtl/edge/shape_ink.v" version="default" library="default"/>'
ANCHOR = '<efx:design_file name="rtl/edge/edge_proc.v" version="default" library="default"/>'


def main():
    if not os.path.exists(XML):
        print("[fix_sources] 找不到 %s" % XML)
        return 1

    with open(XML, "r", encoding="utf-8") as f:
        s = f.read()

    orig = s
    changed = False

    for old in (OLD, OLD_ALT):
        if old in s:
            s = s.replace(old, NEW)
            changed = True

    if "shape_ink.v" not in s:
        if ANCHOR in s:
            s = s.replace(ANCHOR, ANCHOR + "\n        " + NEW)
            changed = True
        else:
            print("[fix_sources] 找不到插入锚点 edge_proc.v, 请手工检查工程")
            return 1

    if changed:
        # Efinity 会把只读文件写回只读状态, 先确保可写
        try:
            os.chmod(XML, 0o666)
        except OSError:
            pass
        with open(XML, "w", encoding="utf-8", newline="\n") as f:
            f.write(s)
        print("[fix_sources] 已修正工程文件列表 -> rtl/edge/shape_ink.v")
    else:
        print("[fix_sources] 文件列表已正确, 无需修改")

    if "shape_rec.v" in s:
        print("[fix_sources] 警告: 仍然残留 shape_rec.v 引用")

    return 0


if __name__ == "__main__":
    sys.exit(main())
