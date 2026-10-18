# benchgate 记录目录

> 由 benchgate 自动生成，请勿手改。源数据：`models/index.jsonl`，共 6 条。生成于 2026-10-17 22:58。

## 当前状态

没有仿真或 gate 记录。

| 类型 | 最近一次 | 结论 | 输入 | 摘要 | 记录 |
|---|---|---|---|---|---|
| 采集 | 2026-10-17 22:58 | 正常 | — | vboost-slow_v_v, vboost-fast_v_v · 附件 2 | `20261017T145816Z_c073` |

## 记录（最新在前，最多 300 条）


### 2026-10-17

| 时间 | 类型 | 结论 | 摘要 | 目的 | 记录 |
|---|---|---|---|---|---|
| 22:58 | 采集 | 正常 | vboost-slow_v_v, vboost-fast_v_v · 附件 2 | 用 CH1 波形算 V_BOOST 是否到 32V、是否高过高边漏极 | `20261017T145816Z_c073` |
| 22:13 | 采集 | 正常 | vboost-ch1_vboost_ch1_v · 附件 1 | 补上 V_BOOST 的 CH1 波形，替代空通道那次采集 | `20261017T141331Z_f1f1` |
| 22:09 | 采集 | 正常 | scope_ch1_avg, scope_ch1_max, scope_ch1_min, pa4_raw 等 6 路 | 转动时对一下示波器 CH1 和升压反馈，看 V_BOOST 有没有到 32V | `20261017T140919Z_d573` |
| 22:05 | 采集 | 有空通道 | scope_ch1 · 空通道 1 | 电机停转时看 V_BOOST 是否到 32V，以便和高边漏极比较 | `20261017T140525Z_8bfb` |
| 15:00 | 采集 | 有空通道 | scope_ch1 · 空通道 1 · V_BOOST | Scope is DC coupled, 10 V/div, 10X, free-running. Capture V_BOOST to read the level and the ripple the screen can resolve. | `20261017T070007Z_b06f` |
| 14:52 | 采集 | 有空通道 | scope_ch1 · 空通道 1 · V_BOOST | CH1 probe is 10X on V_BOOST. Capture the boost output ripple while it regulates and the motor bridge stays off. | `20261017T065257Z_c37d` |
