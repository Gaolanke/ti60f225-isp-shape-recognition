#!/usr/bin/env python3
"""形状识别算法离线对比研究.

目的
----
原工程的识别器是"固定半径 Hough + 固定长度行程 + 5 种长宽比假设"的手工规则,
参数与靶纸尺寸强绑定, 换尺寸/换姿态/换场景就失效. 这里用同一批数据比较三条路线:

  A. 连通域 + 归一化几何特征 + 线性分类器   (本文采用, 纯整数, 无乘法器/除法器阵列)
  B. 同特征 + 小型全连接网络 (int8 量化)
  C. 32x32 边缘图块 + 轻量 CNN (YOLO 蒸馏的"学生网络"形态)

比较维度: 准确率 / 参数量 / 每次推理 MAC 数 / 估计的 FPGA 资源量级.

特征模型与 RTL (rtl/edge/shape_rec 部分) 逐位对齐: 全部用整数运算,
1/4 降采样后按行游程做连通域, 再算 span 面积/周长/外接框/方向直方图.

输出:
  - build/recog/weights.vh   : 量化后的分类器权重 (给 RTL)
  - build/recog/report.md    : 对比报告
  - build/recog/dataset.npz  : 特征数据集
"""
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

RNG = np.random.default_rng(20261101)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "build", "recog")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------------------
# 1. 场景渲染
# ---------------------------------------------------------------------------
FULL_W, FULL_H = 1920, 1080
DS = 4                      # 降采样倍数, 与 RTL 一致
W, H = FULL_W // DS, FULL_H // DS    # 480 x 270

CLASS_NAMES = ["none", "circle", "rect", "triangle"]
NC = len(CLASS_NAMES)


def draw_shape(kind, canvas, cx, cy, r, ang=0.0, aspect=None, thick=2):
    """在 1/4 分辨率画布上画形状轮廓."""
    cx, cy, r, ang = float(cx), float(cy), float(r), float(ang)
    if kind == "circle":
        ry = int(r * (aspect if aspect else 1.0))
        cv2.ellipse(canvas, (int(cx), int(cy)), (int(r), int(ry)),
                    ang, 0, 360, 255, thick)
    elif kind == "rect":
        w = r
        h = r * (aspect if aspect else 1.0)
        box = cv2.boxPoints(((cx, cy), (2 * w, 2 * h), ang))
        cv2.polylines(canvas, [np.int32(box)], True, 255, thick)
    elif kind == "triangle":
        pts = []
        for k in range(3):
            a = math.radians(ang - 90 + 120 * k)
            pts.append([cx + r * math.cos(a), cy + r * math.sin(a)])
        cv2.polylines(canvas, [np.int32(pts)], True, 255, thick)
    elif kind == "plus":
        w = max(4, int(r) // 2)
        cv2.line(canvas, (int(cx - r), int(cy)), (int(cx + r), int(cy)), 255, w)
        cv2.line(canvas, (int(cx), int(cy - r)), (int(cx), int(cy + r)), 255, w)
    elif kind == "line":
        cv2.line(canvas, (int(cx - r), int(cy)),
                 (int(cx + r), int(cy + r * 0.2)), 255, thick)


def add_clutter(canvas, density=0.0, min_len=20, max_len=120):
    if density <= 0:
        return
    n = int(density * 40)
    for _ in range(n):
        x0 = RNG.integers(0, W)
        y0 = RNG.integers(0, H)
        L = RNG.integers(min_len, max_len)
        a = RNG.uniform(0, math.pi)
        x1 = int(x0 + L * math.cos(a))
        y1 = int(y0 + L * math.sin(a))
        cv2.line(canvas, (int(x0), int(y0)), (x1, y1), 255, 1)


def blur_and_thin(edge, ksize=3):
    """模拟真实 Sobel 输出: 膨胀一下再细化的效果用闭运算近似."""
    if ksize <= 1:
        return edge
    k = np.ones((ksize, ksize), np.uint8)
    return cv2.dilate(edge, k, iterations=1)


def make_scene(kind, r=None, ang=None, aspect=None, clutter=0.0, thick=1):
    canvas = np.zeros((H, W), np.uint8)
    if kind != "none":
        r = r if r else RNG.integers(18, 60)
        ang = ang if ang is not None else RNG.uniform(0, 360)
        cx = RNG.integers(r + 10, W - r - 10)
        cy = RNG.integers(int(r * 1.2) + 10, H - int(r * 1.2) - 10)
        draw_shape(kind, canvas, cx, cy, r, ang, aspect, thick)
    add_clutter(canvas, clutter)
    return canvas


# ---------------------------------------------------------------------------
# 2. 特征提取 (与 RTL 逐位对齐的整数模型)
# ---------------------------------------------------------------------------
LABELS = 8
RUN_GAP = 0          # 行游程允许的列间隙(0 = 严格相邻)


def extract_features(edge):
    """edge: HxW uint8 (0/255). 返回每个连通域的特征列表.

    与 RTL 一致:
      * 逐行取游程 (runs), 相邻行游程只要 x 区间重叠(或相距 1)即合并
      * 每行更新外接框 / 周长计数 / span 面积 / 4 方向直方图
      * 标签表只有 LABELS 个, 溢出的一律丢弃
    """
    parent = list(range(LABELS))
    feats = [dict(n=0, x0=10**6, y0=10**6, x1=-1, y1=-1, span=0, maxspan=0,
                  h=[0, 0, 0, 0], used=False, ymin=10**6, ymax=-1)
             for _ in range(LABELS)]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra == rb:
            return ra
        # 保留已有内容较少的一个作为根, 简单起见固定保留较小 id
        if ra < rb:
            parent[rb] = ra
            join(rb, ra)
            return ra
        parent[ra] = rb
        join(ra, rb)
        return rb

    def join(src, dst):
        if not feats[src]["used"]:
            return
        s, d = feats[src], feats[dst]
        d["n"] += s["n"]
        d["x0"] = min(d["x0"], s["x0"])
        d["x1"] = max(d["x1"], s["x1"])
        d["y0"] = min(d["y0"], s["y0"])
        d["y1"] = max(d["y1"], s["y1"])
        d["span"] += s["span"]
        for i in range(4):
            d["h"][i] += s["h"][i]
        d["used"] = True

    prev = []          # [(start, end, label)]
    for y in range(H):
        row = edge[y] > 0
        if not row.any():
            # 行结束: 结算 span
            for (_s, _e, lab) in prev:
                pass
            prev = []
            continue
        idx = np.flatnonzero(row)
        # 生成游程
        runs = []
        s = idx[0]
        p = idx[0]
        for v in idx[1:]:
            if v <= p + 1 + RUN_GAP:
                p = v
            else:
                runs.append((s, p))
                s = v
                p = v
        runs.append((s, p))
        if len(runs) > LABELS * 2:
            runs = runs[:LABELS * 2]

        cur = []
        j = 0
        for (a, b) in runs:
            hit = []
            k = j
            while k < len(prev) and prev[k][0] <= b + 1 + RUN_GAP:
                if prev[k][1] >= a - 1 - RUN_GAP:
                    hit.append(prev[k][2])
                k += 1
            j = max(0, k - 1)
            if not hit:
                # 新标签
                free = [i for i in range(LABELS) if not feats[i]["used"]]
                if not free:
                    continue
                lab = free[0]
                feats[lab]["used"] = True
                feats[lab].update(n=0, x0=10**6, y0=10**6, x1=-1, y1=-1,
                                  span=0, maxspan=0, h=[0, 0, 0, 0],
                                  ymin=10**6, ymax=-1)
            else:
                lab = hit[0]
                for o in hit[1:]:
                    lab = union(lab, o)
            lab = find(lab)
            cur.append((a, b, lab))
            f = feats[lab]
            f["used"] = True
            f["n"] += (b - a + 1)
            f["x0"] = min(f["x0"], a)
            f["x1"] = max(f["x1"], b)
            f["y0"] = min(f["y0"], y)
            f["y1"] = max(f["y1"], y)
            f["ymin"] = min(f["ymin"], y)
            f["ymax"] = max(f["ymax"], y)
        # 行内 span: 每个标签在本行的最左/最右
        per = {}
        for (a, b, lab) in cur:
            if lab not in per:
                per[lab] = [a, b]
            else:
                per[lab][0] = min(per[lab][0], a)
                per[lab][1] = max(per[lab][1], b)
        for lab, (a, b) in per.items():
            feats[lab]["span"] += (b - a + 1)
            if (b - a + 1) > feats[lab]["maxspan"]:
                feats[lab]["maxspan"] = (b - a + 1)
        prev = cur

    out = []
    for i in range(LABELS):
        f = feats[i]
        if not f["used"] or f["x1"] < 0:
            continue
        w = f["x1"] - f["x0"] + 1
        h = f["y1"] - f["y0"] + 1
        if w < 3 or h < 3:
            continue
        out.append(dict(n=f["n"], w=w, h=h, span=f["span"], maxspan=max(1, f["maxspan"]),
                        x0=f["x0"], y0=f["y0"], x1=f["x1"], y1=f["y1"],
                        hist=f["h"]))
    return out


def featurize(comp):
    """把连通域折算成分类器输入 (定点整数, 与 RTL 一致).

    f1 extent      = 256*span/(w*h)      椭圆~200 方~256 三角~125
    f2 compactness = 16*n*n/span         圆~161  方~234  三角~253
    f3 aspect      = 256*w/h
    f4 dir_dom     = 256*max(hist)/sum   多边形方向集中, 圆分散
    f2/f1 用最大 10 位量化, 避免三角形饱和后被截断.
    """
    n, w, h, span = comp["n"], comp["w"], comp["h"], comp["span"]
    if w < 3 or h < 3 or span <= 0:
        return None
    f1 = min(1023, (256 * span) // (w * h))
    f2 = min(1023, (16 * n * n) // max(1, span))
    f3 = min(1023, (256 * w) // h)
    ms = max(1, comp.get("maxspan", w))
    f4 = min(1023, (256 * span) // (ms * ms))
    return [f1, f2, f3, f4]


def label_of(kind):
    return {"none": 0, "circle": 1, "rect": 2, "triangle": 3}[kind]


# ---------------------------------------------------------------------------
# 3. 数据集
# ---------------------------------------------------------------------------
def build_dataset(n_per_class=400, rot_limit=60.0, patch_size=32):
    """每个场景取面积最大的连通域作为样本, 与 RTL 的选优逻辑一致.

    同时输出 32x32 图块, 保证特征路线与 CNN 路线使用**同一批样本、同一划分**,
    这样两者的准确率才可比.
    """
    X, P, y, meta = [], [], [], []
    kinds = ["circle", "rect", "triangle", "none"]
    for kind in kinds:
        made, guard = 0, 0
        while made < n_per_class and guard < n_per_class * 40:
            guard += 1
            if kind == "none":
                sub = RNG.choice(["plus", "line", "clutter"], p=[0.4, 0.25, 0.35])
                if sub == "plus":
                    edge = make_scene("plus", r=RNG.integers(18, 40))
                elif sub == "line":
                    edge = make_scene("line", r=RNG.integers(30, 70))
                else:
                    edge = np.zeros((H, W), np.uint8)
                    add_clutter(edge, 1.0, 15, 90)
            else:
                aspect = RNG.uniform(0.8, 1.25)
                ang = RNG.uniform(-rot_limit, rot_limit)
                edge = make_scene(kind, ang=ang, aspect=aspect,
                                  clutter=RNG.choice([0, 0, 0.1, 0.2]), thick=1)
            comps = extract_features(edge)
            if not comps:
                continue
            comp = max(comps, key=lambda c: c["span"])
            fv = featurize(comp)
            if fv is None:
                continue
            if kind != "none" and comp["span"] < 200:
                continue
            # 同一连通域裁 32x32 图块
            cx0 = (comp["x0"] + comp["x1"]) // 2
            cy0 = (comp["y0"] + comp["y1"]) // 2
            half = int(max(comp["x1"] - comp["x0"], comp["y1"] - comp["y0"]) * 0.75) + 8
            if (cx0 - half < 0 or cy0 - half < 0 or
                    cx0 + half >= W or cy0 + half >= H):
                continue
            crop = edge[cy0 - half:cy0 + half, cx0 - half:cx0 + half]
            patch = cv2.resize(crop, (patch_size, patch_size),
                               interpolation=cv2.INTER_AREA)
            X.append(fv)
            P.append((patch > 40).astype(np.float32) * 255)
            y.append(label_of(kind))
            meta.append(dict(kind=kind, n=comp["n"], w=comp["w"], h=comp["h"],
                             span=comp["span"]))
            made += 1
    return (np.array(X, np.float32), np.array(P, np.float32)[:, None, :, :],
            np.array(y, np.int64), meta)


# ---------------------------------------------------------------------------
# 4. 分类器 A: 线性 (softmax), 定点量化
# ---------------------------------------------------------------------------
def train_linear(X, y, epochs=3000, lr=0.05):
    mu = X.mean(0)
    sd = X.std(0) + 1e-6
    Xn = (X - mu) / sd
    N, D = Xn.shape
    W = np.zeros((D, NC))
    b = np.zeros(NC)
    Y = np.eye(NC)[y]
    for it in range(epochs):
        z = Xn @ W + b
        z -= z.max(1, keepdims=True)
        p = np.exp(z)
        p /= p.sum(1, keepdims=True)
        g = (p - Y) / N
        W -= lr * (Xn.T @ g + 1e-4 * W)
        b -= lr * g.sum(0)
    pred = (Xn @ W + b).argmax(1)
    return W, b, mu, sd, (pred == y).mean()


def quantize_linear(W, b, mu, sd, X, y, q=256):
    """把 (x-mu)/sd 展开成整数形式: score = sum(w_i*x_i) + c
    其中 w_i = W_i/(sd_i) * q, c = b - sum(W_i*mu_i/sd_i) * q
    """
    Wq = np.round(W / sd[:, None] * q).astype(np.int64)
    cq = np.round((b - (W / sd[:, None]).T @ mu) * q).astype(np.int64)
    score = X.astype(np.int64) @ Wq + cq
    pred = score.argmax(1)
    return Wq, cq, (pred == y).mean(), score


# ---------------------------------------------------------------------------
# 5. 分类器 B: 小型全连接 (int8 量化)
# ---------------------------------------------------------------------------
def train_mlp(X, y, hidden=24, epochs=4000, lr=0.05):
    mu = X.mean(0)
    sd = X.std(0) + 1e-6
    Xn = (X - mu) / sd
    N, D = Xn.shape
    W1 = RNG.normal(0, 1 / math.sqrt(D), (D, hidden))
    b1 = np.zeros(hidden)
    W2 = RNG.normal(0, 1 / math.sqrt(hidden), (hidden, NC))
    b2 = np.zeros(NC)
    Y = np.eye(NC)[y]
    for it in range(epochs):
        h = np.maximum(0, Xn @ W1 + b1)
        z = h @ W2 + b2
        z -= z.max(1, keepdims=True)
        p = np.exp(z)
        p /= p.sum(1, keepdims=True)
        g = (p - Y) / N
        gW2 = h.T @ g + 1e-4 * W2
        gb2 = g.sum(0)
        gh = (g @ W2.T) * (h > 0)
        gW1 = Xn.T @ gh + 1e-4 * W1
        gb1 = gh.sum(0)
        W1 -= lr * gW1; b1 -= lr * gb1
        W2 -= lr * gW2; b2 -= lr * gb2
    pred = (np.maximum(0, Xn @ W1 + b1) @ W2 + b2).argmax(1)
    return (W1, b1, W2, b2, mu, sd), (pred == y).mean()


def quantize_mlp(model, X, y, q=64):
    W1, b1, W2, b2, mu, sd = model
    W1q = np.round(W1 / sd[:, None] * q).astype(np.int64)
    b1q = np.round((b1 - (W1 / sd[:, None]).T @ mu) * q).astype(np.int64)
    W2q = np.round(W2 * q).astype(np.int64)
    b2q = np.round(b2 * q * q).astype(np.int64)
    hq = np.maximum(0, X.astype(np.int64) @ W1q + b1q)
    score = hq @ W2q + b2q
    pred = score.argmax(1)
    return (W1q, b1q, W2q, b2q), (pred == y).mean()


# ---------------------------------------------------------------------------
# 6. 分类器 C: 32x32 边缘图块 + 轻量 CNN (numpy 实现, 仅用于对比)
# ---------------------------------------------------------------------------
def make_patches(n_per_class=300, size=32):
    """按连通域外接框裁 32x32 图块, 供 CNN 分支使用."""
    X, y = [], []
    kinds = ["circle", "rect", "triangle", "none"]
    for kind in kinds:
        made, guard = 0, 0
        while made < n_per_class and guard < n_per_class * 40:
            guard += 1
            canvas = np.zeros((H, W), np.uint8)
            if kind == "none":
                add_clutter(canvas, 1.0, 12, 60)
            else:
                r = RNG.integers(22, 42)
                cx = RNG.integers(r + 8, W - r - 8)
                cy = RNG.integers(r + 8, H - r - 8)
                draw_shape(kind, canvas, cx, cy, r,
                           RNG.uniform(-60, 60), RNG.uniform(0.85, 1.2), 1)
            comps = extract_features(canvas)
            if not comps:
                continue
            c = max(comps, key=lambda z: z["span"])
            cx0 = (c["x0"] + c["x1"]) // 2
            cy0 = (c["y0"] + c["y1"]) // 2
            half = int(max(c["x1"] - c["x0"], c["y1"] - c["y0"]) * 0.75) + 8
            if cx0 - half < 0 or cy0 - half < 0 or cx0 + half >= W or cy0 + half >= H:
                continue
            crop = canvas[cy0 - half:cy0 + half, cx0 - half:cx0 + half]
            patch = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
            if (patch > 40).mean() < 0.02:
                continue
            X.append((patch > 40).astype(np.float32) * 255)
            y.append(label_of(kind))
            made += 1
    return np.array(X)[:, None, :, :], np.array(y)


def _im2col(x, k):
    N, C, Hh, Ww = x.shape
    oh, ow = Hh - k + 1, Ww - k + 1
    col = np.stack([x[:, :, i:i + oh, j:j + ow].reshape(N, C, -1)
                    for i in range(k) for j in range(k)], axis=1)
    return col.transpose(0, 2, 1, 3).reshape(N, C * k * k, oh * ow), oh, ow


def _col2im(col, shape, k):
    N, C, Hh, Ww = shape
    oh, ow = Hh - k + 1, Ww - k + 1
    col = col.reshape(N, C, k * k, oh * ow)
    out = np.zeros(shape, np.float32)
    idx = 0
    for i in range(k):
        for j in range(k):
            out[:, :, i:i + oh, j:j + ow] += col[:, :, idx, :].reshape(N, C, oh, ow)
            idx += 1
    return out


def _pool2(x):
    N, C, Hh, Ww = x.shape
    return x.reshape(N, C, Hh // 2, 2, Ww // 2, 2).max(axis=(3, 5))


def _pool2_back(d, shape):
    N, C, Hh, Ww = shape
    out = np.zeros(shape, np.float32)
    d4 = d[:, :, :, None, :, None]
    out = out.reshape(N, C, Hh // 2, 2, Ww // 2, 2)
    out[:] = d4
    return out.reshape(shape)


def train_tiny_cnn(X, y, Xte=None, yte=None, epochs=10, lr=0.03, seed=1):
    """32x32 边缘图块 -> conv5x5(8) -> relu -> maxpool2 -> fc -> 4 类.

    这是"YOLO 蒸馏学生网络"的形态: 输入的 32x32 图块由教师模型的检测框裁出,
    这里用同样的几何增广数据训练, 用于和纯几何特征路线对比精度与开销.
    """
    rng = np.random.default_rng(seed)
    N = len(X)
    X = (X.astype(np.float32) / 255.0).reshape(N, 1, 32, 32)
    Y = np.eye(NC)[y]
    K = 5
    W1 = rng.normal(0, 0.12, (8, 1, K, K)).astype(np.float32)
    b1 = np.zeros(8, np.float32)
    Wf = rng.normal(0, 0.05, (8 * 14 * 14, NC)).astype(np.float32)
    bf = np.zeros(NC, np.float32)
    params = W1.size + b1.size + Wf.size + bf.size
    macs = 8 * 1 * K * K * 28 * 28 + (8 * 14 * 14) * NC

    def forward(xb):
        col, oh, ow = _im2col(xb, K)
        z1 = (W1.reshape(8, -1) @ col).reshape(len(xb), 8, oh, ow) + b1[None, :, None, None]
        a1 = np.maximum(0, z1)
        p1 = _pool2(a1)
        flat = p1.reshape(len(xb), -1)
        z = flat @ Wf + bf
        z -= z.max(1, keepdims=True)
        e = np.exp(z)
        p = e / e.sum(1, keepdims=True)
        return p, dict(col=col, z1=z1, a1=a1, p1=p1, flat=flat, oh=oh, ow=ow)

    for ep in range(epochs):
        perm = rng.permutation(N)
        for i in range(0, N, 32):
            idx = perm[i:i + 32]
            xb, yb = X[idx], Y[idx]
            p, cache = forward(xb)
            g = (p - yb) / len(idx)
            gWf = cache["flat"].T @ g + 1e-4 * Wf
            gbf = g.sum(0)
            dflat = g @ Wf.T
            dp1 = dflat.reshape(cache["p1"].shape)
            da1 = _pool2_back(dp1, cache["a1"].shape)
            dz1 = da1 * (cache["z1"] > 0)
            dz1f = dz1.reshape(len(idx), 8, -1)
            gW1 = (dz1f @ cache["col"].transpose(0, 2, 1)).sum(0).reshape(W1.shape) + 1e-4 * W1
            gb1 = dz1.sum(axis=(0, 2, 3))
            W1 -= lr * gW1
            b1 -= lr * gb1
            Wf -= lr * gWf
            bf -= lr * gbf

    evX = X if Xte is None else (Xte.astype(np.float32) / 255.0).reshape(len(Xte), 1, 32, 32)
    evY = y if yte is None else yte
    corr = 0
    for i in range(0, len(evX), 64):
        p, _ = forward(evX[i:i + 64])
        corr += (p.argmax(1) == evY[i:i + 64]).sum()
    return corr / len(evX), params, macs


# ---------------------------------------------------------------------------
# 7. 主流程
# ---------------------------------------------------------------------------
def train_tree(X, y, depth=6, seed=0):
    from sklearn.tree import DecisionTreeClassifier
    t = DecisionTreeClassifier(max_depth=depth, random_state=seed,
                               min_samples_leaf=8).fit(X, y)
    return t


def export_tree(t, path):
    """把决策树 splice 进 rtl/edge/shape_rec.v 的 TREE_BEGIN/END 之间.

    记录布局: [39]=leaf [38:37]=cls [36:34]=feat [33:18]=thr
              [17:12]=left [11:6]=right
    阈值与特征都是整数; 判定 x <= thr 走左子树, 与 sklearn 一致.
    """
    tr = t.tree_
    n = tr.node_count
    lines = ["    // === 由 tools/recog_study.py 自动生成, 请勿手改 ===",
             "    // [39]=leaf [38:37]=cls [36:34]=feat [33:18]=thr "
             "[17:12]=left [11:6]=right",
             "    function [39:0] tree_node;",
             "        input [5:0] idx;",
             "        begin",
             "            tree_node = 40'd0;",
             "            case (idx)"]
    for i in range(n):
        l = int(tr.children_left[i])
        r = int(tr.children_right[i])
        if l < 0:
            v = tr.value[i][0]
            cls = int(np.argmax(v))
            rec = (1 << 39) | (cls << 37)
        else:
            f = int(tr.feature[i])
            th = max(0, min(65535, int(round(float(tr.threshold[i])))))
            rec = (f << 34) | (th << 18) | (l << 12) | (r << 6)
        lines.append("                6'd%02d: tree_node = 40'h%010X;" % (i, rec))
    lines.append("                default: tree_node = 40'h%010X;" % ((1 << 39) | (0 << 37)))
    lines.append("            endcase")
    lines.append("        end")
    lines.append("    endfunction")
    body = "\n".join(lines)

    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    m0 = src.index("    //=== TREE_BEGIN ===")
    m1 = src.index("    //=== TREE_END ===")
    head = src[:m0]
    tail = src[m1:]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(head + "    //=== TREE_BEGIN ===\n" + body + "\n" + tail)
    return n


def main():
    if cv2 is None:
        print("需要 opencv-python-headless")
        return 2

    print("[1] 生成数据集 (特征 + 图块, 同一批样本) ...")
    X, P, y, meta = build_dataset(400)
    print("    样本 %d, 特征维度 %d, 类别分布 %s" %
          (len(X), X.shape[1], np.bincount(y, minlength=NC)))
    np.savez_compressed(os.path.join(OUT, "dataset.npz"), X=X, y=y,
                        meta=np.array([str(m) for m in meta]))

    idx = RNG.permutation(len(X))
    ntr = int(0.75 * len(X))
    tr, te = idx[:ntr], idx[ntr:]

    print("[2] 线性分类器 ...")
    W, b, mu, sd, _ = train_linear(X[tr], y[tr])
    Wq, cq, accq, _ = quantize_linear(W, b, mu, sd, X[te], y[te])
    acc_lin = float(((((X[te] - mu) / sd) @ W + b).argmax(1) == y[te]).mean())
    print("    test float=%.3f  int=%0.3f" % (acc_lin, accq))

    print("[3] 决策树 ...")
    t = train_tree(X[tr], y[tr], depth=6)
    acc_tree = float(t.score(X[te], y[te]))
    print("    test=%.3f  nodes=%d" % (acc_tree, t.tree_.node_count))
    export_tree(t, os.path.join(ROOT, "rtl", "edge", "shape_rec.v"))

    print("[4] 32x32 图块 CNN (同样的样本与划分) ...")
    accc, nparam, nmac = train_tiny_cnn(P[tr], y[tr], P[te], y[te],
                                        epochs=25, lr=0.05)
    print("    test=%.3f  params=%d  MACs=%d" % (accc, nparam, nmac))

    print("[5] 单特征类均值:")
    for i in range(X.shape[1]):
        vals = [X[y == c, i].mean() for c in range(NC)]
        print("    f%d  %s" % (i + 1, ["%.0f" % v for v in vals]))

    report = {
        "n_samples": int(len(X)),
        "features": ["extent", "compactness", "aspect", "width_solidity"],
        "linear_float_acc": acc_lin,
        "linear_int_acc": float(accq),
        "tree_acc": acc_tree,
        "tree_nodes": int(t.tree_.node_count),
        "cnn_test_acc": float(accc),
        "cnn_params": int(nparam),
        "cnn_macs": int(nmac),
    }
    with open(os.path.join(OUT, "report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n=== 汇总 (同一测试集, 同一批样本) ===")
    print("A  几何特征 + 决策树  acc=%.3f  节点=%d" % (acc_tree, t.tree_.node_count))
    print("A' 几何特征 + 线性    acc=%.3f  MAC=%d" % (accq, X.shape[1] * NC))
    print("B  32x32 CNN          acc=%.3f  MAC=%d  params=%d" % (accc, nmac, nparam))
    return 0


if __name__ == "__main__":
    sys.exit(main())
