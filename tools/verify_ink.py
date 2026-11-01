#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_ink.py -- 用实拍照片验证"墨迹分割 + 填充率分类"这套算法.

RTL (rtl/edge/shape_ink.v) 里做的事, 这里按相同顺序复现一遍:
    4x4 取最暗降采样 -> 3x3 局部均值 - 像素 -> 阈值 -> 3x3 膨胀
    -> 游程 + 并查集连通域 -> fill = span/(h*maxspan) -> 两阈值分类

用法:
    python tools/verify_ink.py <图片目录> [--thr 18] [--show]

输出每个连通域的 bbox / fill / 判类, 以及一张 *_result.png 可视化图.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import recog_study as R          # 复用其中的连通域实现

LABELS = 32
MIN_SPAN = 400
MIN_WH = 14
T_THR = 0.66          # 三角 / 圆 分界
T_RECT = 0.86         # 圆 / 矩形 分界

COLOR = {1: (0, 200, 0), 2: (255, 128, 0), 3: (0, 0, 255)}
TAG = {1: "O", 2: "#", 3: "T"}


def ink_mask(gray, thr=18, win=3):
    """4x4 取最暗 -> 3x3 局部均值 - 像素 -> 阈值 -> 3x3 膨胀."""
    h, w = gray.shape[0] // 4, gray.shape[1] // 4
    d = gray[: h * 4, : w * 4].reshape(h, 4, w, 4).min(axis=(1, 3))
    mean = cv2.blur(d.astype(np.float32), (win, win))
    m = ((mean - d) > thr).astype(np.uint8)
    m = cv2.dilate(m, np.ones((3, 3), np.uint8))
    return d, m


def classify(fill):
    if fill < T_THR:
        return 3
    if fill < T_RECT:
        return 1
    return 2


def analyse(path, thr=18, win=3):
    g = cv2.imread(path, 0)
    if g is None:
        return None
    d, m = ink_mask(g, thr, win)
    R.LABELS = LABELS
    out = []
    for c in R.extract_features(m * 255):
        if c["span"] < MIN_SPAN or c["w"] < MIN_WH or c["h"] < MIN_WH:
            continue
        fill = min(1.0, c["span"] / (c["h"] * max(1, c["maxspan"])))
        if fill < 0.30:
            continue
        out.append(dict(box=(c["x0"], c["y0"], c["x1"], c["y1"]),
                        w=c["w"], h=c["h"], span=c["span"],
                        ms=c["maxspan"], fill=fill, cls=classify(fill)))
    out.sort(key=lambda z: -z["span"])
    return d, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="实拍图目录")
    ap.add_argument("--thr", type=int, default=18)
    ap.add_argument("--win", type=int, default=3)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.src, "*.png")) +
                   glob.glob(os.path.join(args.src, "*.jpg")))
    if not files:
        print("目录里没有图片:", args.src)
        return 1

    out_dir = args.out or os.path.join(args.src, "_result")
    os.makedirs(out_dir, exist_ok=True)

    total_hit = 0
    for p in files:
        res = analyse(p, args.thr, args.win)
        if res is None:
            continue
        d, items = res
        vis = cv2.cvtColor(d, cv2.COLOR_GRAY2BGR)
        for it in items:
            x0, y0, x1, y1 = it["box"]
            cv2.rectangle(vis, (x0, y0), (x1, y1), COLOR[it["cls"]], 1)
            cv2.putText(vis, "%s%.2f" % (TAG[it["cls"]], it["fill"]),
                        (x0, max(9, y0 - 2)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, COLOR[it["cls"]], 1)
        cv2.imwrite(os.path.join(out_dir, os.path.basename(p)), vis)

        text = "  ".join("%s(fill=%.2f %dx%d)" % (TAG[i["cls"]], i["fill"], i["w"], i["h"])
                         for i in items)
        print("%-24s %2d 个: %s" % (os.path.basename(p), len(items), text or "无"))
        total_hit += len(items)

    print("\n合计框出 %d 个目标, 可视化见 %s" % (total_hit, out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
