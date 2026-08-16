# shared/tnb — 主机与从机共用的协议真源

布局对齐 [`tars-io-mux/shared/tnb`](../../tars-io-mux/shared/tnb)。

motor-ctrl 从机经 **I2C1（PB6/PB7）** 与 TARS 主控通信。寄存器映射、PRODUCT_ID、CRC 等待定义在本目录，供：

- `src/firmware/stm32f030/App/` 从机固件
- TARS `App/node_bus/` 或 benchgate 主机工具

## 状态

占位目录；协议文件尚未添加。可扩展 TNB PROFILE 或 motor 专用寄存器块。
