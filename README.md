# motor-ctrl

STM32F030K6（LQFP-32）三半桥 PMSM 主控：PCB + 固件。Hall 闭环 6-step（`hall6`）仍是回退路径；交付的控制是定点 FOC 电流环 + 速度环（`motor_foc_fx.c`），成绩见 [`docs/report/speed-loop-20260914.md`](docs/report/speed-loop-20260914.md)。

目录布局与 [`tars-io-mux`](../tars-io-mux) 一致，便于复用工具链与协作习惯。

```
motor-ctrl/
├── docs/                     # 设计文档、引脚表、协议说明、测试流程
├── models/captured/          # 实测/示波器采数（非 pcb 内 models）
├── pcb/                      # KiCad 工程与 Gerber 输出
├── sim/                      # MATLAB/Simulink FOC 模型 + codegen_stm32/
├── src/                      # 全部固件源码
│   ├── firmware/stm32f030/   # F030K6 可烧录工程（App / Core / HAL）
│   ├── platforms/stm32f030/  # F030 可复用驱动（hall6 / 开环 / PWM / Hall / tick）
│   └── shared/tnb/           # 与主机共用的 I²C / TNB 协议头文件
└── tools/                    # 主机侧脚本、测试工具
```

MATLAB 模型与 `sim/codegen_stm32/` 浮点代码**不**进默认固件（M0 软浮点放不下 20 kHz）。参数真源是 `sim/mc_params.m`（λ = 6.3 mWb，Kt = 0.0378 N·m/A）；改它不会改芯片上的定点环，除非再跑 `gen_code.m`。

## 快速入口

| 路径 | 说明 |
|------|------|
| [`src/firmware/stm32f030/README.md`](src/firmware/stm32f030/README.md) | 固件构建、烧录、默认参数、测试脚本、已知陷阱 |
| [`src/platforms/stm32f030/README.md`](src/platforms/stm32f030/README.md) | 平台层：hall6 闭环、开环、TIM1 PWM 与控制环 |
| [`src/shared/tnb/README.md`](src/shared/tnb/README.md) | 主机–从机协议真源 |
| [`docs/roadmap.md`](docs/roadmap.md) | FOC / 弱磁 / 动态性能的分阶段路线图与验收判据 |
| [`docs/report/speed-loop-20260914.md`](docs/report/speed-loop-20260914.md) | 速度环交付：λ、Kt、wrap 成绩、转矩曲线 |
| [`docs/measurement-validity.md`](docs/measurement-validity.md) | 实测前自检清单、静默限幅一览、无效实验模式 |

改动 PWM 相关代码前，先读 `src/firmware/stm32f030/README.md` 的
[已知陷阱](src/firmware/stm32f030/README.md#已知陷阱)——低边常通的 `CCR` 取值、中心对齐的
`RCR`、DTG 非线性编码、半桥板 VOUT 电容，这几条都是实测中代价很大才定位的。

## docs/

设计文档、引脚对照、协议说明、测试流程。建议首批文档：

- `pins.md` — F030K6 与半桥板、Hall、分流电阻接线
- `i2c-protocol.md` — 从机寄存器（或 TNB 扩展）与 TARS 主控对接
- `bringup.md` — 上电、SWD、开环与 hall6 闭环验证步骤
