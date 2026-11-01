#!/usr/bin/env python3
"""Convert a binary PPM (P6) to PNG without third-party dependencies."""

import argparse
import struct
import zlib


def read_token(data, pos):
    while pos < len(data):
        if data[pos] in b" \t\r\n":
            pos += 1
        elif data[pos:pos + 1] == b"#":
            while pos < len(data) and data[pos] not in b"\r\n":
                pos += 1
        else:
            break
    start = pos
    while pos < len(data) and data[pos] not in b" \t\r\n":
        pos += 1
    return data[start:pos], pos


def read_ppm(path):
    data = open(path, "rb").read()
    magic, pos = read_token(data, 0)
    if magic != b"P6":
        raise ValueError("only binary PPM (P6) is supported")
    width_b, pos = read_token(data, pos)
    height_b, pos = read_token(data, pos)
    maxval_b, pos = read_token(data, pos)
    width = int(width_b)
    height = int(height_b)
    maxval = int(maxval_b)
    if maxval != 255:
        raise ValueError("only 8-bit PPM is supported")
    if pos < len(data) and data[pos:pos + 2] == b"\r\n":
        pos += 2
    elif pos < len(data) and data[pos:pos + 1] in (b"\r", b"\n"):
        pos += 1
    pixels = data[pos:pos + width * height * 3]
    if len(pixels) != width * height * 3:
        raise ValueError("truncated PPM pixel data")
    return width, height, pixels


def chunk(tag, payload):
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    )


def write_png(path, width, height, pixels, scale):
    if scale < 1:
        raise ValueError("scale must be >= 1")
    if scale != 1:
        rows = []
        stride = width * 3
        for y in range(height):
            src = pixels[y * stride:(y + 1) * stride]
            row = b"".join(src[x:x + 3] * scale for x in range(0, stride, 3))
            for _ in range(scale):
                rows.append(b"\x00" + row)
        raw = b"".join(rows)
        width *= scale
        height *= scale
    else:
        raw = b"".join(
            b"\x00" + pixels[y * width * 3:(y + 1) * width * 3]
            for y in range(height)
        )
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as fp:
        fp.write(png)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ppm")
    parser.add_argument("--output")
    parser.add_argument("--scale", type=int, default=1)
    args = parser.parse_args()
    width, height, pixels = read_ppm(args.ppm)
    output = args.output or (args.ppm[:-4] + ".png" if args.ppm.lower().endswith(".ppm") else args.ppm + ".png")
    write_png(output, width, height, pixels, args.scale)
    print(f"{args.ppm} -> {output} ({width * args.scale}x{height * args.scale})")


if __name__ == "__main__":
    main()
