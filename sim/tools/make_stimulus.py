#!/usr/bin/env python3
"""Convert a bitmap into the edge/direction stimulus consumed by tb_shape_roi.

Output (per image, 960x240 ROI):
  <out>_edge.hex : 28800 words, bit i = pixel (8k+i) is an edge
  <out>_dir.hex  : 28800 words, nibble i = {band8[2:0], quad}
  <out>_view.png : preview (gray + green edges + red direction ticks)

Direction encoding matches rtl/edge/edge_proc.v:
  band8 = 8-way quantization of |Gy|/|Gx| (0 = gradient nearly horizontal)
  quad  = sign(gx) XOR sign(gy)
"""
import argparse, os
import cv2
import numpy as np

ROI_W, ROI_H = 960, 240
BAND_T = np.array([0.203, 0.414, 0.668, 1.0, 1.496, 2.414, 5.031], np.float32)
BAND_UV = [(256, 0), (251, 46), (237, 98), (217, 143),
           (181, 181), (143, 217), (98, 237), (46, 251)]


def load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    if img.ndim == 3 and img.shape[2] == 4:
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        rgb = img[:, :, :3].astype(np.float32)
        img = (rgb * alpha + 255.0 * (1.0 - alpha)).astype(np.uint8)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def fit_into(img, w, h, margin=0.10):
    """Letterbox the image into w x h on a white canvas."""
    ih, iw = img.shape
    avail_w, avail_h = int(w * (1 - 2 * margin)), int(h * (1 - 2 * margin))
    scale = min(avail_w / iw, avail_h / ih)
    nw, nh = max(1, int(round(iw * scale))), max(1, int(round(ih * scale)))
    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC
    small = cv2.resize(img, (nw, nh), interpolation=interp)
    canvas = np.full((h, w), 255, np.uint8)
    x0, y0 = (w - nw) // 2, (h - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = small
    return canvas, (x0, y0, nw, nh)


def degrade(img, blur=0, noise=0, grad=0, seed=1):
    rng = np.random.default_rng(seed)
    out = img.astype(np.float32)
    if grad:
        yy, xx = np.mgrid[0:out.shape[0], 0:out.shape[1]].astype(np.float32)
        g = 1.0 + grad * ((xx / out.shape[1]) - 0.5) + grad * 0.5 * ((yy / out.shape[0]) - 0.5)
        out *= g
    if blur:
        k = int(blur) * 2 + 1
        out = cv2.GaussianBlur(out, (k, k), blur)
    if noise:
        out += rng.normal(0, noise, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def sobel_dir(gray):
    gx = cv2.Sobel(gray.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
    ax = np.abs(gx).astype(np.float32)
    ay = np.abs(gy).astype(np.float32)
    ratio = ay / np.maximum(ax, 1e-6)
    band = np.searchsorted(BAND_T, ratio, side='left').astype(np.uint8)
    np.clip(band, 0, 7, out=band)
    quad = ((gx < 0) != (gy < 0)).astype(np.uint8)
    return band, quad, gx, gy


def write_hex(path, words, width):
    with open(path, 'w', encoding='ascii', newline='\n') as f:
        for w in words:
            f.write(f"{w:0{width}X}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--canny-low', type=int, default=60)
    ap.add_argument('--canny-high', type=int, default=140)
    ap.add_argument('--blur', type=float, default=0.0)
    ap.add_argument('--noise', type=float, default=0.0)
    ap.add_argument('--grad', type=float, default=0.0)
    ap.add_argument('--margin', type=float, default=0.10)
    ap.add_argument('--invert', action='store_true', help='white shapes on black')
    ap.add_argument('--dilate-y', type=int, default=0,
                    help='vertical edge thickening radius (photo regression)')
    args = ap.parse_args()

    gray = load_gray(args.image)
    if args.invert:
        gray = 255 - gray
    canvas, box = fit_into(gray, ROI_W, ROI_H, args.margin)
    canvas = degrade(canvas, args.blur, args.noise, args.grad)
    edges = cv2.Canny(canvas, args.canny_low, args.canny_high, L2gradient=True)
    if args.dilate_y > 0:
        kernel = np.ones((args.dilate_y * 2 + 1, 1), np.uint8)
        edges = cv2.dilate(edges, kernel)
    band, quad, gx, gy = sobel_dir(canvas)

    edge_words = [0] * (ROI_W * ROI_H // 8)
    dir_words = [0] * (ROI_W * ROI_H // 8)
    ys, xs = np.nonzero(edges)
    for y, x in zip(ys.tolist(), xs.tolist()):
        idx = y * ROI_W + x
        edge_words[idx >> 3] |= 1 << (idx & 7)
        dir_words[idx >> 3] |= (int(band[y, x]) << 1 | int(quad[y, x])) << ((idx & 7) * 4)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_hex(args.out + '_edge.hex', edge_words, 8)
    write_hex(args.out + '_dir.hex', dir_words, 8)

    view = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    view[edges > 0] = (0, 220, 0)
    for y, x in zip(ys[::7].tolist(), xs[::7].tolist()):
        b = int(band[y, x])
        ux, uy = BAND_UV[b]
        sg = -1 if quad[y, x] else 1
        x2 = int(x + 0.06 * ux)
        y2 = int(y + 0.06 * uy * sg)
        cv2.line(view, (x, y), (x2, y2), (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(args.out + '_view.png', view)
    print(f"{args.image}: edges={int(edges.sum())} box={box} -> {args.out}_{{edge,dir}}.hex")


if __name__ == '__main__':
    main()


