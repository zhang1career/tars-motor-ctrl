# benchgate 记录目录

> 由 benchgate 自动生成，请勿手改。源数据：`models/index.jsonl`，共 83 条。生成于 2026-10-14 17:47。

## 当前状态

没有仿真或 gate 记录。

| 类型 | 最近一次 | 结论 | 输入 | 摘要 | 记录 |
|---|---|---|---|---|---|
| 采集 | 2026-10-14 17:47 | 正常 | — | tick_cyc_max, tick_cyc_min, tick_cyc_edge, tick_cyc_steady 等 6 路 · stm32f030-tick | `20261014T094740Z_1446` |

## 记录（最新在前，最多 300 条）


### 2026-10-14

| 时间 | 类型 | 结论 | 摘要 | 目的 | 记录 |
|---|---|---|---|---|---|
| 17:47 | 采集 | 正常 | tick_cyc_max, tick_cyc_min, tick_cyc_edge, tick_cyc_steady 等 6 路 · stm32f030-tick | Confirm whether a current-loop hall-edge tick can exceed 2400 cycles. Forced edge measured 1811, steady interpolating ticks 1206 to 1220, bridge stayed off. | `20261014T094740Z_1446` |
| 17:25 | 采集 | 正常 | id_mean_lsb, iq_mean_lsb, n_points, hall 等 12 路 | 复跑静止、iq 0 的电流环窗，并把清零之后的最长一拍算作电流环自己的。 | `20261014T092534Z_0dba` |
| 17:24 | 采集 | 正常 | code, stopped, i24, hall 等 5 路 · 附件 1 | 复跑完成标准第1条：转子静止、iq 0；并把六步和电流环的最长一拍分开 | `20261014T092418Z_4c3c` |
| 17:18 | 采集 | 正常 | tick_overrun, tick_total, tick_cyc_max, tick_cyc_avg 等 9 路 | 新固件第一次起转进电流环时的节拍诊断：超时次数、最长一拍、平均，以及电流环窗里的 id/iq 均值。 | `20261014T091801Z_a12f` |
| 17:16 | 采集 | 正常 | code, stopped, i24, hall 等 5 路 · 附件 1 | 新固件（常见拍 1167）第一次起转：10% 六步交接到电流环，iq 0，看读状态、节拍超时和最长一拍 | `20261014T091617Z_3436` |
| 16:53 | 采集 | 正常 | tick_cyc_min, tick_cyc_max, tick_cyc_common | Whole-tick cycles after packing hall state and removing the unread direction store, 24 V on, bridge off. | `20261014T085345Z_6756` |
| 16:51 | 采集 | 正常 | tick_cyc_min, tick_cyc_max, tick_cyc_common | Whole-tick cycles after splitting the current PI into several calls, 24 V on, bridge off. | `20261014T085128Z_188c` |
| 15:47 | 采集 | 正常 | tick_cyc_min, tick_cyc_max, tick_cyc_common | Whole-tick cycle count of the handover current loop after the straight-line tick, with 24 V off and the bridge disabled. | `20261014T074756Z_b5e9` |
| 14:36 | 采集 | 正常 | tick_first, tick_min, tick_max_later, tick_common 等 5 路 | Measure the whole current-loop tick in clock cycles, with the 24 V bus off and the bridge disabled. | `20261014T063627Z_404e` |
| 14:29 | 采集 | 正常 | div1_cyc, div2_cyc, pow2_cyc, q_1000 等 5 路 | Time the reciprocal divider that replaces libgcc division on the current-loop tick, with the 24 V bus off and the bridge disabled. | `20261014T062902Z_19ce` |
| 13:21 | 采集 | 正常 | seg0, seg1, seg2, seg3 等 8 路 · 附件 1 | 用查表和缓存缩短电流环之后，再量整拍还剩多少周期 | `20261014T052142Z_6a75` |
| 12:56 | 采集 | 正常 | seg0, seg1, seg2, seg3 等 8 路 · 附件 1 | 把电流环一步拆成四段，看 2300 周期耗在哪 | `20261014T045616Z_ce91` |
| 12:29 | 采集 | 正常 | seg0, seg1, seg2, seg3 等 8 路 · 附件 1 | 确认跳过六步里的电压计算之后，电流环本身还要多少周期 | `20261014T042944Z_897c` |
| 12:27 | 采集 | 正常 | seg0, seg1, seg2, seg3 等 8 路 · 附件 1 | 六步期间量四段耗时，确认整拍超过 50 微秒导致看门狗复位 | `20261014T042725Z_4183` |
| 12:04 | 采集 | 正常 | win_a_ok, win_b_ok, win_c_ok, win_a_fail 等 11 路 · 附件 1 | Check whether one-byte status reads succeed after the I2C slave is allowed to stretch the clock, with the tick stopped, with a short tick, and with the current-loop code running. | `20261014T040448Z_ee4e` |
| 11:16 | 采集 | 正常 | tick_off_ok, tick_off_fail, light_tick_avg, light_tick_ok 等 9 路 · 附件 1 | Compare one-byte status reads with the 50 us tick stopped, with a short tick, and with the current-loop tick, all with the bridge off. | `20261014T031633Z_f62e` |
| 11:10 | 采集 | 正常 | reads_ok, reads_fail, avg_tick, window_overrun_delta 等 10 路 · 附件 1 | Repeat the one-byte status read after the current loop is steady, on the NOSTRETCH firmware, to see if the read still fails when the tick is about 2300 cycles. | `20261014T031030Z_fcd7` |
| 10:50 | 采集 | 正常 | reads_ok, reads_fail, rtt_min_ms, rtt_max_ms 等 10 路 · 附件 1 | See whether a one-byte status read completes while the bridge switches at a fixed common-mode duty and the rotor stays still. | `20261014T025008Z_ce87` |
| 10:16 | 采集 | 正常 | six_state, six_mode, six_cyc_max, six_overrun 等 8 路 | 桥已开、节拍还轻时读一个字节。发出时最长 1520、超时 0，这个字节没有在六步结束前读完 | `20261014T021625Z_91e9` |
| 10:08 | 采集 | 正常 | six_cyc_max, six_overrun, six_state, six_mode 等 8 路 | 想在六步阶段读状态，但这次读跨过了交接，不能用来区分原因 | `20261014T020814Z_17b7` |
| 03:06 | 采集 | 正常 | state_max, stat_ok, stat_n, i24 等 5 路 | I2C 从机改成允许拉低时钟后，起转进电流环再读 10 次状态，看还失不失败 | `20261013T190632Z_4e42` |
| 02:54 | 采集 | 正常 | avg_cyc, steady_max_cyc, burst_max_cyc, overrun 等 9 路 | 电流环稳态节拍约 2178，最长约 2192，10 次读状态全部失败 | `20261013T185441Z_cb7f` |
| 02:22 | 采集 | 正常 | state_max, mode_max, hall_edges, fault 等 10 路 | 10% 起转时用调试口看状态。进了电流环，但 mot stat 回复是 FAIL | `20261013T182249Z_2f79` |
| 01:57 | 采集 | 正常 | hall6-2_hall_raw, hall6-2_step, hall6-2_ticks_since_edge, hall6-2_kick 等 8 路 · 附件 1 | 自举没结束时踢腿不再跳步。10% 起转第一次换相在 39 毫秒，霍尔按 5、4、6、2、3、1 走，没有过流 | `20261013T175755Z_3174` |
| 01:56 | 采集 | 正常 | code, stopped, i24 | 自举没结束时不再跳过踢腿第一步，再看 10% 起转 | `20261013T175632Z_f214` |
| 01:56 | 采集 | 正常 | hall6-1_hall_raw, hall6-1_step, hall6-1_ticks_since_edge, hall6-1_kick 等 9 路 · 附件 1 | 10% 起转的霍尔迹：第一步在 59 毫秒才出现，而且是第 2 步，第 1 步被自举窗口吞掉 | `20261013T175605Z_5211` |
| 01:54 | 采集 | 正常 | code, stopped, i24 | 起转前武装 hall6 迹，看踢腿第一步是不是落在自举窗口里 | `20261013T175418Z_20c5` |
| 01:43 | 采集 | 正常 | code, flt, phase, i_lsb 等 14 路 | 10% 起转，零点已减掉。六步还没交到电流环就软件过流，桥被锁住 | `20261013T174338Z_48fb` |
| 01:41 | 采集 | 正常 | code, stopped, i24 | 零点已减掉，节拍在起转前保持关闭，10% 交接进电流环，iq 指令 0 | `20261013T174145Z_bc34` |
| 01:36 | 采集 | 正常 | code, stopped, i24 | 零点已减掉后，10% 六步交接进电流环，iq 指令 0 | `20261013T173604Z_2fbf` |
| 01:27 | 采集 | 正常 | ready, zero_u, zero_v, zero_w 等 9 路 | 零点采集放在电机节拍里，空闲时节拍没开，所以没有采到，没有起转 | `20261013T172713Z_4856` |
| 01:19 | 采集 | 正常 | a1_overrun, a1_ticks, a1_avg_cyc, a1_max_cyc 等 15 路 | I2C 中断改到电机节拍下面之后，不开桥三窗看读状态还会不会撑过 50 微秒 | `20261013T171934Z_1680` |
| 01:09 | 采集 | 正常 | a1_overrun, a1_avg_cyc, a1_max_cyc, b_overrun 等 11 路 | 读状态的计算已离开 I2C 中断后，不开桥三窗看节拍是否还被撑过 50 微秒 | `20261013T170927Z_d260` |
| 01:09 | 采集 | 正常 | ocp_hold, ocp_kind, moe, i24 等 6 路 | 软件过流后锁住桥：强制跳闸，读状态不得重新送电 | `20261013T170926Z_57f0` |

### 2026-10-13

| 时间 | 类型 | 结论 | 摘要 | 目的 | 记录 |
|---|---|---|---|---|---|
| 23:44 | 采集 | 正常 | code, stopped, i24 | 完成标准第1条：观察用累加已关，10% | `20261013T154438Z_7f15` |
| 15:27 | 采集 | 正常 | code, stopped, i24 | 抓看门狗复位原因：10%，诊断记录含节拍差值 | `20261013T072736Z_8b53` |
| 14:39 | 采集 | 正常 | code, stopped, i24 | 完成标准第1条：10% 重跑；上次电流环接手后单片机复位，跑前已清复位标志 | `20261013T063925Z_93cd` |
| 14:10 | 采集 | 正常 | code, stopped, i24 | 完成标准第1条：起转占空比改为10%后，转子静止、q轴指令0，看 id 和 iq 平均 | `20261013T061017Z_a10f` |
| 13:30 | 采集 | 正常 | code, stopped, i24 | 完成标准第1条重试：vsweep 六组正常后，转子静止、q轴指令0，看 id 和 iq 平均 | `20261013T053041Z_be04` |
| 13:26 | 采集 | 正常 | code, state, hall, hall_begin 等 6 路 · 附件 1 | 起转在霍尔2报堵转后，静止扫六组上下管，重点看第二块板下管（U→V、W→V） | `20261013T052626Z_58b6` |
| 13:19 | 采集 | 正常 | code, stopped, i24 | 完成标准第1条：转子静止、q轴指令0，看 id 和 iq 平均 | `20261013T051942Z_e02f` |
| 12:43 | 采集 | 正常 | i_b3_80, i_b3_120, i_b3_160, i_b1_80 等 9 路 | 换第三块板 UCC27211 后复测。固定脉宽保持，回路：3 用第1块下管，1/2 用第3块下管。CCR 80/120/160 期望 0.025/0.091/0.197 A。 | `20261013T044339Z_82f3` |
| 12:01 | 采集 | 正常 | i_b2_80, i_b2_120, i_b2_160, i_b1_80 等 9 路 | 换第二块板 UCC27211 后，排线已恢复。固定脉宽保持，回路：1/2 用第3块下管，3 用第1块下管。CCR 80/120/160 期望 0.025/0.091/0.197 A。 | `20261013T040102Z_f66a` |
| 01:24 | 采集 | 正常 | ipwm080, ipwm0120, ipwm0160, ipwm180 等 7 路 | 第一、二块板排线对调后，两路 PWM 对第三相下管的母线电流 | `20261012T172436Z_b640` |
| 01:11 | 采集 | 正常 | iU80, iU120, iU160, iV80 等 10 路 | 三块板 HIN 都加 220pF 后，三相高边分别开通的母线电流 | `20261012T171136Z_49c1` |

### 2026-10-12

| 时间 | 类型 | 结论 | 摘要 | 目的 | 记录 |
|---|---|---|---|---|---|
| 16:47 | 采集 | 正常 | i48b, i80b, i120b, i160b 等 5 路 | HIN 加 220pF 后复测 CCR 48/80/120/160 母线电流 | `20261012T084754Z_93b6` |
| 16:44 | 采集 | 正常 | i48, i80, i120, i160 等 5 路 | 第一块板 HIN 对 GND 加 220pF 后，无示波器，CCR 48/80/120/160 母线电流 | `20261012T084450Z_6761` |
| 15:30 | 采集 | 正常 | i160, iend | CH2 接回第三相 VOUT 且地夹悬空，CCR160 | `20261012T073058Z_6128` |
| 15:28 | 采集 | 正常 | i160, iend | 再次测 CCR160 第一相第三相下管的母线电流 | `20261012T072829Z_4d8e` |
| 15:12 | 采集 | 正常 | i160, iend | CH2 探针离开 HIN 后，CCR160 母线电流仍为满值 | `20261012T071253Z_c973` |
| 15:10 | 采集 | 正常 | i160, iend | CH2 探针离开 HIN 后，CCR160 第一相第三相下管的母线电流 | `20261012T071005Z_d35e` |
| 14:56 | 采集 | 正常 | i160, iend, hin_n, full · 附件 1 | CH1 第一相 VOUT、CH2 第一块板 HIN，CCR160，1.2ms 高分辨率 | `20261012T065603Z_472a` |
| 12:58 | 采集 | 正常 | i160, iend, pulse · 附件 1 | 恢复 CH1 地夹、去掉 CH2 地夹，CCR160 两相 VOUT | `20261012T045817Z_687a` |
| 12:47 | 采集 | 正常 | i160, iend, pulse · 附件 1 | 去掉 CH1 地夹，CH2 地夹仍在，CCR160 两相 VOUT | `20261012T044752Z_42fb` |
| 12:03 | 采集 | 正常 | i160, iend, pulse · 附件 1 | 第1步：坏状态下第一相和第三相 VOUT，CCR160 | `20261012T040346Z_e8c3` |
| 11:40 | 采集 | 正常 | i160, iend | CH2 地夹再换位置后，CCR160 第一相第三相下管的母线电流 | `20261012T034013Z_4ee2` |
| 11:39 | 采集 | 正常 | i160, iend | CH2 地夹再换位置后，CCR160 第一相第三相下管的母线电流 | `20261012T033905Z_1e02` |
| 11:37 | 采集 | 正常 | i160, iend | CH2 地夹换位置后，CCR160 第一相第三相下管的母线电流 | `20261012T033746Z_acd8` |
| 11:34 | 采集 | 正常 | i160, iend | CH2 地夹换位置后，CCR160 第一相第三相下管的母线电流 | `20261012T033410Z_1619` |
| 11:31 | 采集 | 正常 | i160, iend | 重新接上示波器 CH2 后，CCR160 第一相第三相下管的母线电流 | `20261012T033128Z_d5cb` |
| 11:28 | 采集 | 正常 | i160, iend | 改地线并拆除示波器后，CCR160 第一相第三相下管的母线电流 | `20261012T032836Z_b03e` |
| 02:01 | 采集 | 正常 | i160, iend | 确认两地夹连通后只接 CH1、断开 CH2，CCR160 第一相第三相下管的母线电流 | `20261011T180118Z_c176` |
| 01:31 | 采集 | 正常 | i160, iend | 只拔掉 CH1 地夹，CH2 地夹仍在，CCR160 第一相第三相下管的母线电流 | `20261011T173138Z_3199` |
| 01:30 | 采集 | 正常 | i160, iend | 只拔掉 CH2 地夹，CH1 地夹仍在，CCR160 第一相第三相下管的母线电流 | `20261011T173016Z_9b08` |
| 01:15 | 采集 | 正常 | i160, iend | 两只示波器地夹重新接上后，CCR160 第一相第三相下管的母线电流 | `20261011T171527Z_157a` |
| 01:12 | 采集 | 正常 | i160, iend | 拔掉两个示波器地夹后，CCR160 第一相第三相下管的母线电流 | `20261011T171243Z_cd8b` |
| 00:56 | 采集 | 正常 | i160e, iende, vgs, vout · 附件 1 | CCR160 5us/格 栅极与 VOUT | `20261011T165639Z_492b` |
| 00:51 | 采集 | 正常 | i160, iend, vgs, vout · 附件 3 | CCR160 第一相，示波器看 Q1 栅极对地和 VOUT 对地，探头 10 倍 | `20261011T165156Z_18ce` |

### 2026-10-11

| 时间 | 类型 | 结论 | 摘要 | 目的 | 记录 |
|---|---|---|---|---|---|
| 23:56 | 采集 | 正常 | i80, i120, i160, iend | V_BOOST 外供 32V 后，静止相间窄脉冲的母线电流是否回到计算值 | `20261011T155631Z_053c` |
| 16:17 | 采集 | 正常 | p0l2, p1l2, p2l0, p2l1 | 12 伏、80 计数，三块半桥分别做开关相，看短脉冲下高边是否还能输出 | `20261011T081741Z_96e0` |
| 16:10 | 采集 | 正常 | pwm0_low2, pwm1_low2, pwm2_low0 | 12 伏下三块半桥轮流做 160 计数的开关相，比较母线电流，区分共性问题还是单板问题 | `20261011T081007Z_5eee` |
| 15:38 | 采集 | 正常 | i48, i80, i120, i160 | 母线 12 伏，自举先充后第一相 48/80/120/160 计数、第三相拉低，看母线电流 | `20261011T073803Z_a082` |
| 15:35 | 采集 | 正常 | i48, i80, i120, i160 | 母线降到 6 伏，自举先充后第一相 48/80/120/160 计数、第三相拉低，看母线电流 | `20261011T073540Z_b415` |
| 15:34 | 采集 | 正常 | i48, i100, i150, i200 | 母线降到 6 伏，自举先充后，第一相加脉宽、第三相拉低，看母线电流是否按 1.3 欧的预期上升 | `20261011T073458Z_db49` |
| 15:30 | 采集 | 正常 | i48, i60, i80 | 母线改到 12 伏后，第一相 48、60、80 个计数、第三相拉低，看母线电流 | `20261011T073055Z_729a` |
| 15:28 | 采集 | 正常 | i48, i60 | 自举先充 40 毫秒后，第一相脉宽 48 和 60 个计数、第三相拉低，看 24 伏母线电流 | `20261011T072829Z_e21e` |
| 14:18 | 采集 | 正常 | v | 万用表量第一块半桥 V_PWR 对 GND | `20261011T061843Z_eef4` |
| 14:17 | 采集 | 正常 | v_pwr | 万用表量第一块半桥 J1 的 V_PWR 对 GND | `20261011T061702Z_3ae9` |
| 14:07 | 采集 | 正常 | v, ccr, i24 | 万用表量第一根和第三根，80 计数 | `20261011T060701Z_97f4` |
| 13:54 | 采集 | 正常 | v, ccr, i24 | 万用表量第一根和第三根，48 计数 | `20261011T055402Z_0d1b` |
| 13:22 | 采集 | 正常 | code, state, hall, hall_begin 等 6 路 · 附件 1 | 静止扫电压，死区补偿关闭，看各相指令电压和相电流是否成正比 | `20261011T052247Z_13ce` |
| 13:21 | 采集 | 正常 | code, state, i24 | 静止扫电压，死区补偿关闭，看各相指令电压和相电流是否成正比 | `20261011T052158Z_66a2` |
| 13:17 | 采集 | 正常 | code, state, i24 | 静止扫电压，死区补偿关闭，看各相指令电压和相电流是否成正比 | `20261011T051721Z_e1b1` |
