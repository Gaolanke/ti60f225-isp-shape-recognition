#!/usr/bin/env python3
"""Compose print-like target sheets from the downloaded web shape artwork."""
import os, cv2, numpy as np

WEB = r'E:\FPGA\FPGA_MAX\sim\images\web'
OUT = r'E:\FPGA\FPGA_MAX\sim\images\src'
W, H = 960, 240


def load(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise SystemExit('missing ' + path)
    if img.ndim == 3 and img.shape[2] == 4:
        a = img[:, :, 3:4].astype(np.float32) / 255.0
        img = (img[:, :, :3].astype(np.float32) * a + 255.0 * (1 - a)).astype(np.uint8)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def silhouette(gray):
    """Fill the outer contour so the printed shape has one clean boundary."""
    _, bw = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = np.full_like(gray, 255)
    if cnts:
        cv2.drawContours(out, [max(cnts, key=cv2.contourArea)], -1, 0, thickness=cv2.FILLED)
    ys, xs = np.nonzero(out < 128)
    return out[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def place(canvas, shape, cx, cy, target_h):
    h, w = shape.shape
    s = target_h / h
    nw, nh = max(1, int(round(w * s))), target_h
    r = cv2.resize(shape, (nw, nh), interpolation=cv2.INTER_AREA)
    x0, y0 = int(cx - nw / 2), int(cy - nh / 2)
    roi = canvas[y0:y0 + nh, x0:x0 + nw]
    m = r < 128
    roi[m] = 0
    return canvas


def save(img, name):
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, name + '.png'), img)
    print(name, img.shape, 'dark px', int((img < 128).sum()))


def blank():
    return np.full((H, W), 255, np.uint8)


circle = silhouette(load(os.path.join(WEB, 'Ionicons_ellipse-outline.svg.png')))
rect   = silhouette(load(os.path.join(WEB, 'Rectangle_outline_Pinhead_icon.svg.png')))
tri    = silhouette(load(os.path.join(WEB, 'triangle_up_cc0.png')))
plus   = silhouette(load(os.path.join(WEB, 'plus_Plus_sign_svg.png')))

for name, shp, h, cx in [('circle', circle, 160, 480), ('rect', rect, 130, 480),
                         ('tri', tri, 180, 480), ('plus', plus, 160, 480)]:
    c = blank()
    place(c, shp, cx, 120, h)
    save(c, name + '_web')

c = blank()
place(c, circle, 130, 120, 160)
place(c, rect,   370, 110, 140)
place(c, tri,    690, 115, 180)
save(c, 'sheet_web')

# Plus sign next to the three positives: it must stay undetected.
c = blank()
place(c, circle, 110, 120, 150)
place(c, rect,   330, 110, 130)
place(c, tri,    550, 115, 170)
place(c, plus,   790, 120, 150)
save(c, 'sheet_plus_web')

