# v8.12 彩色画面红色边缘增强

日期：2026-10-31

## 问题

- 普通彩色叠加模式默认使用“按梯度方向着色”，只有竖直边是红色，水平边为绿色、斜边为黄/青色，看起来红色边缘很少。
- 模式 4 的红色 Sobel 边只有 1 像素，再向右扩 1 像素；在 1080p 彩色画面上仍显得细、断、弱。

## 修改

1. 默认边缘颜色改为统一红色（`color_mode = 2'd1`）。KEY1 长按仍可循环红/白/绿/方向彩。
2. 模式 4 红色显示阈值由 `max(i_th_high + 12, 24)` 降为 `max(i_th_high + 4, 16)`。
3. 使用已有的 3x3 Sobel 幅值窗做下/右各 1 像素的二维扩张，把 1 像素轮廓变成约 2 像素连续红边；没有增加行缓存。
4. 识别仍使用原 `i_th_low` 低阈值，并把扩张后的红边同步送入识别器，保证“屏幕红边 = 识别边”，不降低识别灵敏度。
5. 24 级低对比块的显示阈值仍被拒绝（`faint_red=0`），不会重新出现皮肤/衣服整片染红。

## 验证

```text
all -Strict                      PASS
tb_sobel_mode                    PASS
uploaded-photo paper_target      PASS
uploaded-photo tall target       PASS
Efinity map / interface / pnr    PASS
Efinity pgm (bitstream)          PASS
```

模式 4 仿真结果：`red_no_edge=0`、`faint_red=0`、`faint_edge=128`。

## 本版资源

```text
LUT4      21596
FF        11823
SRL8      1303
RAM10     157
DPRAM10   4
DSP48     4
```

## 产物

```text
outflow\ti60f225_oob.bit
时间   2026-10-31 20:11:02
长度   2690994 bytes
SHA256 57966FF9A6748FDFD19070700B93982AB2E0F52B66BBDEA61C3B75C61F563130
```

烧录命令：

```bat
build\flash_jtag.bat
```
