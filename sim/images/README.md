# 网图模型测试集（sim/images）

这些图片只用于**仿真回归**，不参与综合，也不会进入固件。原始素材来自
Wikimedia Commons 的公开授权文件；`src/` 和 `gen/` 是在其基础上合成的
“打印靶纸”与边缘/方向激励。许可与原始页面记录在 `web/sources.json`。

## 原始素材（web/）

| 文件 | 形状 | 作者 | 许可 | 来源 |
|---|---|---|---|---|
| `Ionicons_ellipse-outline.svg.png` | 圆（正样本） | Ionic Team | MIT | https://commons.wikimedia.org/wiki/File:Ionicons_ellipse-outline.svg |
| `Rectangle_outline_Pinhead_icon.svg.png` | 矩形（正样本） | Quincy Morgan | CC0 | https://commons.wikimedia.org/wiki/File:Rectangle_outline_Pinhead_icon.svg |
| `triangle_up_cc0.png` | 三角形（正样本） | Quincy Morgan | CC0 | https://commons.wikimedia.org/wiki/File:Triangle_up_outline_Pinhead_icon.svg |
| `plus_Plus_sign_svg.png` | 加号（负样本） | Indolences | Public domain | https://commons.wikimedia.org/wiki/File:Plus_sign.svg |

`Circle_outline.png.png` 是带坐标轴的教学图，不适合做形状靶纸，已弃用；
`Red_circle_frame_transparent.svg.png` 为公有领域备用圆环。

> 早期下载时误把矩形图复制成了 `Triangle_up_outline_Pinhead_icon.svg.png`。
> 该文件与矩形素材哈希完全相同，已删除。真正的三角形素材是
> `triangle_up_cc0.png`（对应上方 CC0 链接），不要按文件名误删。

## 合成靶纸（src/）

`tools/compose_sheet.py` 把上述线稿填充成实心轮廓（模拟打印/手绘的实心图形），
按 960x240 的识别 ROI 排布：

- `circle_web.png` / `rect_web.png` / `tri_web.png` / `plus_web.png`：单形状
- `sheet_web.png`：圆 + 矩形 + 三角形（对应现场靶纸布局）
- `sheet_plus_web.png`：上述三者 + 加号，加号必须**不**触发任何识别

## 仿真激励（gen/）

`tools/make_stimulus.py` 对合成图做轻量“相机化”（σ=0.8 模糊、σ=2 噪声、
10% 亮度梯度），再执行 Sobel + Canny，输出：

- `<name>_edge.hex`：28800 个 32-bit 字，bit i = 该像素是边缘
- `<name>_dir.hex`：28800 个 32-bit 字，每 4 bit 一个 `{band8[2:0], quad}`
- `<name>_view.png`：边缘（绿）+ 方向（红）预览

编码与 `rtl/edge/edge_proc.v` 的 `i_edge` / `i_dir16` 完全一致。

## 复现

从零重建素材和激励：

```powershell
python sim\tools\compose_sheet.py
python sim\tools\make_stimulus.py --image sim\images\src\sheet_web.png `
       --out sim\images\gen\sheet_web --blur 0.8 --noise 2.0 --grad 0.10
```

运行全部 6 个网图用例（脚本会重新编译 `tb_shape_web.vvp`）：

```powershell
cd sim
.\run_web_tests.ps1 -Frames 12
```

单个形状（圆：强制 asp=4，走 10 档半径扫描）：

```powershell
vvp sim\out\tb_shape_web.vvp +imgtest `
    +imgfile=sim\images\gen\circle_web +img_asp=4 +img_frames=12 +expect=1
```

`+expect` 是位掩码：bit0=圆、bit1=矩形、bit2=三角形；`0` 表示该图不允许
出现任何识别结果。`plus_web` 与 `sheet_plus_web` 中的加号都必须保持零命中。
