# 第三方组件与许可说明

本仓库中由项目作者编写的应用逻辑、测试平台与脚本采用根目录 `LICENSE`
中的 MIT 许可证。以下第三方组件不纳入该 MIT 授权，仍适用其原始许可和版权声明：

- `rtl/csi_rx_controller.sv`、`rtl/efx_dsi_tx.sv`、`rtl/dvi_tx/`、`ip/`
  中包含 Efinix 工具生成或厂商提供的 IP，版权归 Efinix Inc. 及其许可方所有。
  文件中的原始版权和许可头未被删除。
- `rtl/ddr3_controller/` 来源于 Elitestek 的 DDR3 控制器，版权归 Elitestek, Inc.
  所有，适用文件内的原始许可。
- `rtl/cam_i2c_ctrl/i2c_master/` 基于 OpenCores/Intel 许可的 I2C Master，
  适用文件内的原始版权与许可声明。
- `sim/images/web/` 中的测试图片来自 Wikimedia Commons，具体作者、许可与来源记录在
  `sim/images/README.md` 和 `sim/images/web/sources.json`。
- 硬件数据手册、厂商参考 PDF 等未随仓库分发；请从器件厂商获取。

如果需要将本工程用于商业产品，请分别确认上述厂商 IP、工具许可证和板卡资料的授权范围。

