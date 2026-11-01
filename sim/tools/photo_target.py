#!/usr/bin/env python3
'''Build recognition regressions from the uploaded printed target photo.

The photo contains a square, a tall rectangle, a left-pointing triangle and a
circle.  The composite sheet exercises the square, triangle and circle in the
same frame.  The tall rectangle gets an isolated image: the production
classifier emits one box at a time, so two simultaneous rectangles would make
the regression ambiguous rather than stronger.
No replacement shape is drawn: the regression keeps the photographed line
thickness and small wobble.
'''
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'sim' / 'images' / 'photo' / 'uploaded_target_20261030.jpg'
OUT = ROOT / 'sim' / 'images' / 'src' / 'paper_target.png'
TALL_OUT = ROOT / 'sim' / 'images' / 'src' / 'paper_target_tall.png'
PREVIEW = ROOT / 'sim' / 'images' / 'src' / 'paper_target_preview.png'

# Fractions of the original 810x1440 photo, selected around each printed shape.
SHAPES = {
    'square': (0.020, 0.190, 0.325, 0.365),
    'tall_rect': (0.055, 0.515, 0.340, 0.745),
    'triangle': (0.490, 0.155, 0.825, 0.360),
    'circle': (0.535, 0.525, 0.900, 0.735),
}

# The tall rectangle sits next to a soft paper crease.  A slightly stricter
# threshold keeps the printed outline while excluding the crease as one large
# connected blob; the other three crops are clean at the original threshold.
THRESHOLDS = {'tall_rect': 115}


def load_gray(path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise SystemExit(f'cannot read {path}')
    return image


def clean_printed_line(gray, threshold=135):
    '''Keep the main printed line and discard small paper/table specks.'''
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        raise SystemExit('no printed line found')
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    kept_line = np.where(labels == largest, 0, 255).astype(np.uint8)
    ys, xs = np.nonzero(kept_line < 128)
    return kept_line[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def place(canvas, shape, cx, cy, target_h):
    height, width = shape.shape
    scale = target_h / height
    resized = cv2.resize(shape, (max(1, round(width * scale)), target_h),
                         interpolation=cv2.INTER_AREA)
    _, resized = cv2.threshold(resized, 160, 255, cv2.THRESH_BINARY)
    x0 = int(round(cx - resized.shape[1] / 2))
    y0 = int(round(cy - target_h / 2))
    roi = canvas[y0:y0 + target_h, x0:x0 + resized.shape[1]]
    roi[resized < 128] = 0


def main():
    image = load_gray(SOURCE)
    height, width = image.shape
    crops = {}
    for name, (x0f, y0f, x1f, y1f) in SHAPES.items():
        x0, y0 = int(width * x0f), int(height * y0f)
        x1, y1 = int(width * x1f), int(height * y1f)
        crops[name] = clean_printed_line(
            image[y0:y1, x0:x1], THRESHOLDS.get(name, 135))

    canvas = np.full((240, 960), 255, np.uint8)
    place(canvas, crops['square'], 130, 120, 150)
    place(canvas, crops['triangle'], 400, 120, 165)
    place(canvas, crops['circle'], 690, 120, 160)

    tall_canvas = np.full((240, 960), 255, np.uint8)
    place(tall_canvas, crops['tall_rect'], 480, 120, 180)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT), canvas)
    cv2.imwrite(str(TALL_OUT), tall_canvas)
    cv2.imwrite(str(PREVIEW), cv2.resize(
        canvas, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    print(f'photo regression: {SOURCE.name} -> {OUT.name}, '
          f'{TALL_OUT.name}, shapes=' + ','.join(crops))


if __name__ == '__main__':
    main()
