# FPGA_MAX 固件仿真环境

本目录面向仓库根目录（v8.12 仿真回归）的图像处理固件，使用
Icarus Verilog 直接编译真实 RTL，不修改 `rtl/`。当前覆盖板级图像链路的
三个核心部分：

- `rtl/sensor_clipper.v`：SC431HAI2 原始 1081x640 到 1080x480 的窗口裁剪；
- `rtl/edge/edge_proc.v`：灰度、前置滤波、Sobel、NMS、双阈值滞回、分屏/叠加；
- 同文件内的 `shape_proc`：圆/矩形/三角形识别与叠加。

全顶层 `ti60f225_oob_top.v` 暂时不在 Icarus 中仿真，原因见“仿真边界”。

## 快速开始

```powershell
cd sim

# 全部测试（edge_proc + edge_proc/shape_proc 链路）
.\run_sim.ps1 -Test all

# 只跑边缘处理
.\run_sim.ps1 -Test edge

# 只跑边缘 + 形状识别链路
.\run_sim.ps1 -Test chain

# 生产尺度形状识别长回归（320x240、大半径椭圆 + 16px 网格负例）
.\run_sim.ps1 -Test scale -Strict

# 网图模型测试（Wikimedia 线稿 -> 打印靶纸 -> Sobel/Canny 激励）
.\run_web_tests.ps1

# 1920 宽右半屏 ROI 回归（圆/矩形/三角形 + 左半屏负例，约 4~5 分钟）
.\run_sim.ps1 -Test roi -Strict

# 8-bit Gamma ROM 全值域逐项检查
.\run_sim.ps1 -Test gamma -Strict

# BGGR 去马赛克四色场回归（红/蓝/绿/白）
.\run_sim.ps1 -Test debayer -Strict

# SC431HAI2 原始 1081x640 -> 1080x480 窗口裁剪回归
.\run_sim.ps1 -Test clipper -Strict

# 生成 VCD 并用 GTKWave 打开
.\run_sim.ps1 -Test wave

# 清理 out/ 与 wave/ 中的生成文件
.\run_sim.ps1 -Test clean

# 把 WARN 也当作失败（用于严格回归）
.\run_sim.ps1 -Test all -Strict
```

> **固定流程**：任何 RTL 或参数修改后，必须先运行
> `.\run_sim.ps1 -Test all -Strict`；涉及形状识别时再运行 `-Test scale -Strict`
> 和 `-Test roi -Strict`。只有全部 PASS 后，才进入 Efinity 编译和烧录。
> 用户自行编译固件，本目录不启动 Efinity。

脚本会自动查找：

1. `$env:IVERILOG`；
2. 工程内 `tools\iverilog\bin\iverilog.exe`；
3. `sim\tools\iverilog\bin\iverilog.exe`；
4. `C:\Users\<user>\Downloads\Ti60F225_DemoBoard_v4\tools\iverilog\bin\iverilog.exe`；
5. PATH 中的 `iverilog.exe`。

本机当前使用 Icarus Verilog 12，路径为
`%USERPROFILE%\Downloads\Ti60F225_DemoBoard_v4\tools\iverilog\bin`。
GTKWave 位于同目录树的 `gtkwave\bin\gtkwave.exe`。

## 测试内容

### `tb_edge_proc.v`

驱动 64x48 的合成 `{B,G,R}` 视频，自检 6 组用例：

| 用例 | 检查点 |
| --- | --- |
| `gray_conversion` | `R=200,G=100,B=50` 应得到 `0x707070`；分屏右半屏无边缘时为黑 |
| `median_impulse_reject` | 黑底孤立白点经过 3x3 中值后不产生边缘 |
| `bypass_impulse_control` | 同一白点在直通模式下产生边缘，证明中值用例不是“死链路” |
| `step_vertical` | 竖直阶跃产生边缘；测量边缘相对彩色图的水平延迟 |
| `step_horizontal` | 水平阶跃产生边缘；测量垂直延迟 |
| `checkerboard_edges` | 8x8 棋盘边界都有边缘、无 X、`TOP_BLANK/COL_BLANK` 生效 |

同时检查每帧 `o_de` 有效拍数 = 64x48、每行一个 `o_hs`、每帧一个 `o_vs`。

### `tb_edge_chain.v`

把 `edge_proc` 的输出接到 `shape_proc`：

 - `chain_passthrough`：`i_en=0` 时与三拍延迟后的边缘输出逐像素完全一致；
 - `chain_shape_enabled`：棋盘图连续 8 帧打开识别，输出仍为 3072 拍、
   无 X，且形状命中必须为 0；
 - `chain_constant_frame`：纯色帧打开识别，检查累加器/Hough 清空路径不产生 X；
 - `chain_rect_positive`：合成矩形必须只触发矩形分类；
 - `chain_circle_positive`：合成竖直椭圆必须只触发圆分类；
 - `chain_circle_offgrid`：椭圆半径故意落在两档扫描半径之间，仍须识别，
   并检查输出框中心/半径没有偏到目标之外；
 - `chain_circle_clutter`：低对比棋盘纹理上的椭圆必须仍能识别，且不得
   误判成矩形/三角形；
 - `chain_triangle_positive`：合成正立三角形必须只触发三角形分类。

### `tb_shape_scale.v`（可选长回归）

直接驱动生产参数下的 `shape_proc`，使用 320x240 帧、椭圆半径 78/102、扫描
半径 40/56/.../184：

 - 先跑 16px 网格纹理负例，覆盖完整 10 档半径扫描，要求圆/矩形/三角形均不锁定；
 - 再跑一个半径落在扫描档之间的椭圆正例，要求圆框中心误差不超过 12 像素，
   半径落在目标附近，且不再把大椭圆的一段弧误锁成小圆。

这个测试用于标定顶层真实参数，运行约半分钟，不并入 `-Test all`；日常改 RTL 仍以
`-Test all -Strict` 为快速门槛，涉及形状参数时再额外跑 `-Test scale -Strict`。

### `tb_shape_roi.v`（1920 宽 ROI 长回归）

直接使用顶层生产参数，并把 `i_draw_x0=960` 固定为右半屏 ROI：

 - 右半屏 55×86 椭圆必须锁定，中心/横向半径/纵向半径均有收紧断言；
 - 右半屏 110×150 矩形和底边 100、高约 150 的三角形都必须识别；
 - 左半屏放置同尺寸椭圆时，ROI 边缘计数和三类形状锁定必须全部为 0；
 - 测试包含 `+ellipse_only` / `+rect_only` 调试入口，用于单独定位某一类。

运行约 4~5 分钟，不并入 `-Test all`；凡是修改 ROI、Hough、椭圆纵横比或画框
尺寸，都必须额外跑本测试。

### 网图模型测试（`sim/images`）

`sim/images/gen/` 下是用公开授权的网图线稿合成的打印靶纸，再经过
σ=0.8 模糊、σ=2 噪声和 10% 亮度梯度后生成的 `_edge.hex` / `_dir.hex`。
这些激励直接送入 `shape_proc`，覆盖“圆、矩形、三角形、加号负样本”四类：

```powershell
cd sim
.\run_web_tests.ps1
```

也可以单独运行一个用例（`+expect` 是位掩码：bit0=圆、bit1=矩形、
bit2=三角形，0 表示不允许任何识别）：

```powershell
vvp sim\out\tb_shape_web.vvp +imgtest `
    +imgfile=sim\images\gen\circle_web +img_asp=4 +img_frames=12 +expect=1

vvp sim\out\tb_shape_web.vvp +imgtest `
    +imgfile=sim\images\gen\plus_web +img_asp=4 +img_frames=12 +expect=0
```

原始素材来源与许可见 `sim/images/README.md`。加号是负样本：它不得触发
圆/矩形/三角形中的任何一类。

### `tb_gamma_correction.v`

驱动 8-bit `gamma_correction`，把全部 256 个输入值同时应用到 R/G/B 三通道，
并要求输出与 `gamma_2p2_8bit.mem` 逐项完全一致。该测试已并入 `-Test all`。

### `tb_debayer_bg.v`

驱动 `debayer_top_2to1` 的 BGGR 打包输入（偶像素低字节、奇像素高字节），
分别送入红、蓝、绿、白四个纯色场。断言中心区域平均通道关系：

```text
red   -> R 最大
blue  -> B 最大
green -> G 最大
white -> R/G/B 互相接近
gain  -> 1.25/1.00/0.75 白场必须满足 R>G>B
```

旧版 `raw_to_rgb` 会把红/蓝场都输出成 `80/110/80` 的发绿结果；修复后为
`200/80/20`、`20/80/200`。最后一个非等增益白场专门检查
`rgb_gain_v1` 的 BGGR 通道映射。该测试已并入 `-Test all`。

### `tb_sensor_clipper.v`

按厂商 ILA 捕获到的 SC431HAI2 原始节奏，连续驱动 1081 条 MIPI 行，每条行
640 个有效 4-pixel beat。硬断言 `sensor_clipper` 只输出：

```text
1080 条有效行
每条行前 480 个 beat（1920 像素）
第 1081 条行完全不输出
```

这是 v8.0 修复紫屏、逐行红蓝噪点和闪帧的关键回归。该测试已并入 `-Test all`。

## 当前仿真结果

当前快速回归 `-Test all -Strict` 与可选生产尺度回归 `-Test scale -Strict` 的结果：

 - `tb_edge_proc`：**PASS**，6 组用例，0 条 WARN；
 - `tb_edge_chain`：**PASS**，8 组用例；棋盘连续 8 帧无误检，矩形/圆/
   非整档椭圆/棋盘背景椭圆/三角形均只触发本类分类，圆框中心与半径有硬断言。
 - `tb_shape_scale -Strict`：**PASS**；9300 条网格纹理边缘零形状锁定，
   半径 78/102 的椭圆恢复为精确的 `center=(160,120), r=78/102`。
 - v7.2 只修复 Efinity 综合多驱动，不改变算法参数；上述两组回归结果与 v7.1
   一致，仍为 PASS。
 - v7.3 把 `out_line_mem` 从条件异步读改为同步 RAM + 下一地址预取；上述回归
   仍全部 PASS，且 14 张 PPM 与 v7.2 修改前逐字节一致。
 - v7.4 把 Gamma 大表改为 256×8 同步 ROM，并修正椭圆纵横比与实测画框：
   `-Test all -Strict`、`-Test scale -Strict`、`-Test roi -Strict` 全部 PASS；
   ROI 目标 55×86 的输出为精确的 `center=(1440,120), r=55/86`。
 - v7.5 修复 BGGR 去马赛克行相位与 3 拍控制对齐：`tb_debayer_bg -Strict`
   五组颜色/增益测试 PASS，`-Test all -Strict`、`-Test scale -Strict`、`-Test roi -Strict`
   仍全部 PASS。
 - v7.9 修复竖方图跨帧累加，并给旋转三角形增加内部空域门控：
   `-Test all -Strict` 与 `-Test roi -Strict` PASS。
 - v8.0 恢复厂商 `sensor_clipper` 并补入工程 XML：新增
   `tb_sensor_clipper -Strict` PASS，输出严格为
   `1081x640 raw cropped to 1080x480 active`；`-Test all -Strict`、
   `-Test scale -Strict`、`-Test roi -Strict` 仍全部 PASS。
 - v8.7 修复宽椭圆 Hough 参数角、负 `dy` 符号扩展、累加器同址读写旁路，
   并增加圆弧守卫、45 度斜向边缘计数、实测半径比例门控和独立三角保持时间。
   最终仿真结果（被测 RTL SHA-256 `563F2C...68BA`）：
   `all -Strict` PASS，`chain -Strict` 8/8 PASS，`scale -Strict` PASS，
   `roi -Strict` PASS，`target_sheet` PASS（30 帧，`far=0`），
   6 个网图用例（含加号负样本）全部 PASS。

生成文件在 `sim/out/`：

| 文件 | 说明 |
| --- | --- |
| `gray_split.ppm/.png` | 灰度 + 分屏 |
| `median_impulse.ppm/.png` | 中值滤波抑制孤立点 |
| `bypass_impulse.ppm/.png` | 直通对照 |
| `step_vertical.ppm/.png` | 竖直阶跃边缘 |
| `step_horizontal.ppm/.png` | 水平阶跃边缘 |
| `checkerboard.ppm/.png` | 8x8 棋盘边缘 |
| `chain_passthrough.ppm/.png` | 边缘链路透传 |
| `chain_shape.ppm/.png` | 打开形状识别后的链路输出 |
| `chain_constant.ppm/.png` | 纯色帧 + 形状识别 |
| `chain_rect.ppm/.png` | 矩形正例与青色框 |
| `chain_circle.ppm/.png` | 椭圆正例与黄色框 |
| `chain_circle_offgrid.ppm/.png` | 半径不在扫描档上的椭圆压力用例 |
| `chain_circle_clutter.ppm/.png` | 棋盘纹理上的椭圆压力用例 |
| `chain_triangle.ppm/.png` | 三角形正例与品红框 |
| `*.log` | 每个测试台的完整输出 |

PNG 由 `tools/ppm_to_png.py` 生成，默认 4 倍最近邻放大，便于查看。

## 当前验证重点（v8.7）

### 已通过的关键约束

1. **像素与节拍**：`edge_proc` 每帧输出 64×48 个 `o_de` 有效拍，每行一个
   `o_hs`，每帧一个 `o_vs`；链路打开/关闭形状识别时拍数一致。
2. **方向编码**：竖直阶跃输出红色 `0000ff`、`dir16=0`；水平阶跃输出绿色
   `00ff00`、`dir16=14`；棋盘不再出现方向总线错位。
3. **帧首白线**：`TOP_BLANK=8`，灰度和边缘输出在行 0..7 均为黑色；
   水平阶跃测试检查该区域 `000000/0`。
4. **形状识别**：矩形、圆、三角形正例分别只触发本类分类；纯棋盘连续 8 帧
   零命中；棋盘纹理上的椭圆仍能锁定圆框，且输出框中心/半径进入硬断言。

### v8.7 几何一致性约束

1. **宽椭圆按参数角投票**：5 种纵横比（1.0 / 1.3125 / 1.5625 / 0.625 /
   0.5）各有 8 个 `(cos t, sin t)` 表项。宽椭圆端部不再形成独立小圆峰；
   正圆表与旧表逐项相同。
2. **弧窗守卫只放宽采样**：`CIRC_ARC_GUARD=16`，但四角空域仍严格落在候选框
   内，矩形角不能靠守卫混入。
3. **斜向边缘计数抗棋盘**：`CIRC_DIAG_MIN=16`。8×8 棋盘可命中 6/8 位置弧窗，
   但 `diag=0`，因此不再误报圆；生产尺度椭圆的 `diag=77~249`。
4. **实测半径下限 75%**：`meas_size_ok` 拒绝 `rx=81` 对 `rx=136` 这种端帽
   塌缩，靶纸中的 `far circle at (341,150)` 不再出现。
5. **三角形独立保持**：`TRI_HOLD=90`，三角框不会提前消失后被圆重新解释。
6. **加号负样本**：`plus_web`（只有加号）的 `circ=0 rect=0 tri=0` 是硬断言；
   `sheet_plus_web` 只证明加号与三类正例共存时正例仍全部命中。

关键实测值：

```text
target_sheet  tri=(1588,30)-(1880,210)  rect=(1050,30)-(1400,169)
              circ=(420,150) r=160/80   far=0
roi 高椭圆   (1440,120) r=55/86        roi 宽椭圆 (1440,120) r=86/55
web circle   (1440,120) r=65/65        web rect   (1352,68)-(1526,171)
web tri      (1361,70)-(1518,191)      web plus   零命中
```

### v7.1 识别参数标定与补强

1. **椭圆候选不再只看峰值**：每个候选需要 Hough 峰值达到 ROI 边缘数的
   `5/64`，后续帧再通过椭圆八方向弧窗口（8 个窗口至少命中 6 个）和四角
   纹理上限（生产尺度 64 像素）联合确认。
2. **矩形/三角形交叉抑制**：圆候选中心若落在三角候选框附近，不参与圆锁定；
   三角斜边容差已参数化（`TRI_TOL_PX`），小分辨率仿真不会再被 12 像素固定
   容差放大。
3. **三角内部空域**：三角候选内部必须基本无边缘；棋盘纹理或桌面纹理不能再
   仅凭两条斜边命中凑出三角框。
4. **Hough 量化容差**：生产图使用 8px 累加器 bin 和 16px 半径步进，候选中心/
   半径可能偏离若干像素。顶层的弧采样半宽由 4 调到 8、四角上限由 32 调到 64；
   320x240 生产尺度回归证明该组合能接受半径 78/102 的非整档椭圆，同时对
   9300 条网格纹理边缘保持零形状锁定。
5. **框位置回归**：小分辨率链路测试现在直接检查圆框中心 `(cx,cy)` 和半径，
   防止“分类命中但框偏到目标外”这种假通过。

### v7.2 综合多驱动修复

1. Efinity 在 elaboration 阶段发现 `circ_corner_hits[15]` 同时被独立计数器块和
   主形状状态机复位段驱动（`VDB-1000/VDB-1001`）。
2. 删除主状态机复位段的 `circ_corner_hits <= 16'd0;`，保留独立计数器块的完整
   复位、逐帧清零和命中递增逻辑；算法与显示行为不变。
3. 修复后重新运行 `-Test all -Strict`（6+8 项全 PASS）和
   `-Test scale -Strict`（PASS），并做按模块划分的跨 `always` 赋值静态检查。
4. Icarus 不会把这类多驱动当错误，因此 **仿真 PASS 不能替代 Efinity
   elaboration**；后续新增寄存器时需同时关注综合日志。

### v7.3 行缓存同步读资源修复

1. Efinity map 后 PNR 报容量不足：LUT/FF 均约 9.7 万，而器件单类容量约 6.08 万；
   `u_edge_proc` 独占约 7.9 万 FF / 6.35 万 LUT。
2. 根因是 `out_line_mem`（2048×39 位）在 `if (s22_de)` 内异步读，被推断为
   logic memory；不是形状识别导致。
3. 改为无条件同步读，并在有效拍前预取下一地址，保持原有逐像素对齐时序。
4. 回归结果：上述 6+8 项和 `-Test scale -Strict` 全部 PASS；14/14 张 PPM
   与修改前的 `sim/out/v72_baseline/` 逐字节一致。

### v7.4 Gamma ROM 与椭圆框贴合

1. Gamma 1024×10 异步查找表改为 256×8 同步 ROM，保留 1 拍延迟和全部
   `valid/hs/vs` 行为；`tb_gamma_correction` 对全部 256 个输入值 × 3 通道
   做 bit-exact 检查。
2. 椭圆扫描加入 21/16 与 25/16 两档纵横比，适配斜视打印圆。
3. 已确认候选的真实边界只在四个宽正方向窗口内测量；某个轴没有覆盖完整时
   独立回退该轴，避免半个椭圆把框心拉偏。
4. 极值顶到搜索窗边界的轴一律不采用，防止棋盘/桌面纹理被当成目标边界。
5. 新增 1920 宽 ROI 回归，右半屏目标尺寸与左半屏负例都进入硬断言。

### 历史问题记录（v7.1 已修复）

以下内容是 v6.7 时代的仿真发现，保留用于追溯；当前严格回归已不再出现这些
失败，且对应的像素/方向/棋盘误检用例均已成为硬断言。

1. **每行第一个有效像素为 0**：v7.1 回归中不再有该 WARN。
2. **边缘结果相对彩色图延迟 5 像素/行**：v6.3 像素链路对齐及后续同步修复后，
   当前竖直/水平阶跃用例均通过。
3. **边缘标志与方向总线不同步**：当前方向颜色和 `dir16` 断言均通过。
4. **`shape_proc` 在棋盘/纯色帧误检**：棋盘连续 8 帧和纯色帧现在都要求
   `shape_hit_events=0`，压力回归已通过。

## 仿真边界与后续扩展

Icarus 目前只跑 `edge_proc + shape_proc`，不跑全顶层，原因是：

- `rtl/dvi_tx/*.v_encrypted.v` 是加密 RTL，Icarus 无法解密；
- `rtl/csi_rx_controller.sv`、`efx_dsi_tx.sv` 是较大的 SystemVerilog/IP 模型；
- DDR3 控制器、MIPI CSI RX、PLL/SerDes 等 Efinity IP 没有可用的 Icarus 行为模型；
- 完整 1920x1080 帧在 Icarus 中仿真时间和内存开销都较大。

建议的扩展顺序：

1. 保持当前 `edge_proc + shape_proc` 回归作为算法主回归；
2. 为 `frame_buffer_V4` 的 AXI 读写接口写一个行为级 DDR 模型，仿真
   `frame_buffer` 的 bank 乒乓/冻结逻辑；
3. 用 Efinity Simulation 或 ModelSim/Questa 跑 `ti60f225_oob_top.v`，并对
   `dvi_encoder`、`csi_rx_controller`、DDR3 PHY 使用厂商仿真库；
4. 若必须用 Icarus，可为加密 DVI、CSI RX、DDR3 顶层分别写 stub，只保留
   视频时序和像素通路，但那时验证的是“系统连接”而不是完整固件。

## 目录结构

```
sim/
├─ run_sim.ps1              # 编译、运行、PPM->PNG、波形入口
├─ README.md                # 本文件
├─ tb/
│  ├─ tb_edge_proc.v        # edge_proc 自检
│  ├─ tb_edge_chain.v       # edge_proc + shape_proc 链路
│  ├─ tb_shape_scale.v      # 生产尺度 shape_proc 长回归
│  ├─ tb_shape_roi.v        # 1920 宽右半屏 ROI 长回归
│  ├─ tb_sobel_mode.v       # 模式 4 显示边与识别输入逐像素一致性
│  ├─ tb_gamma_correction.v # Gamma ROM 全值域回归
│  ├─ tb_debayer_bg.v       # BGGR 红/蓝/绿/白场回归
│  └─ tb_sensor_clipper.v   # 1081x640 -> 1080x480 窗口裁剪回归
├─ tools/
│  └─ ppm_to_png.py         # 无第三方依赖的 PPM(P6)->PNG
├─ out/                     # vvp、log、ppm、png
└─ wave/                    # VCD
```

## 关于 BRAM 初始化

Icarus 不会自动把推断出的 RAM 初始化为 0，而 Efinity 的 RAM10/EBR 上电为 0。
测试台在每个用例开始时显式清零：

- `lb0_mem/lb1_mem/flb0..flb3_mem/mb0_mem/mb1_mem/sb0_mem/sb1_mem/hb_mem`
  （`edge_proc`）；
- `acc_mem/vf_a/vf_b`（`shape_proc`）。

Gamma ROM 由 `$readmemh` 读取 `rtl/gamma_conrrection/gamma_2p2_8bit.mem`；
测试台也用同一文件生成期望表，因此不存在两套手工维护的数据。

这只模拟硬件上电状态，不改变 RTL 行为。

## 增加新用例

在对应测试台中：

1. 调用 `prepare_test`（清 RAM + 复位 + 清捕获）；
2. 填充 `src[]` 并设置 `i_prefilt/i_overlay/i_split_col` 等配置；
3. 先跑一帧 `capture_on=0` 预热行缓存，再 `clear_capture`、`capture_on=1`
   跑测量帧；
4. 用 `cap[]/cap_e[]/cap_dir[]` 做断言，必要时调用 `write_ppm`。

修改 RTL 时不要绕过仿真：如果接口、时序或识别裁决发生变化，必须同步更新
测试台和本 README，并先通过 `-Test all -Strict`；形状参数变化还要加跑
`-Test scale -Strict`。

v8.6 起，模式 4 的识别输入必须与屏幕红色 Sobel 边一致。修改显示/识别通路后
还要运行：

```text
.\run_sim.ps1 -Test sobel -Strict
```

该测试要求 `red_count == edge_count` 且 `edge_mismatch == 0`。
