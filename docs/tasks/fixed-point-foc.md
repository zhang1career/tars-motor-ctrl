# 任务：定点 Hall-FOC（`motor_foc_fx.c`）

**面向接手执行的 agent。本文件是完整规格，不需要你做数值设计判断——所有标度、常数、
判据都已算好写死。遇到本文没写的情况，停下来问，不要自己发明。**

前置阅读（顺序不要变）：

1. [`../measurement-validity.md`](../measurement-validity.md) 第 1 节（通电前自检）与 3.10（估算的教训）
2. [`../roadmap.md`](../roadmap.md) 3.3.2（为什么必须定点）、3.5（定点实现规范）、4.1 与 5.1（电流与角度的标定结果）

---

## 1. 目标

在 `src/platforms/stm32f030/motor_foc_fx.{c,h}` 实现定点 Hall-FOC，
**能在 20 kHz 控制 ISR 里运行**。

现有的浮点实现 `motor_foc.c` **不要修改**，也不要删除——它是本任务的对照基准（第 5 节）。

### 为什么是定点

不是偏好，是实测强制的。浮点版实测 **16173 周期**，而一拍只有 2400 周期；
软浮点在本平台**每次运算 169 周期**（`ctrl_bench` 的 `fmul100` 案例实测）。
详见 roadmap 3.3.2。

### 硬性预算

`ctrl_bench` 实测控制 ISR 的其余部分：无边沿的一拍 283 周期，有 hall 边沿的一拍 563 周期。

> **`MotorFocFx_Step()` 的中位数必须 < 1837 周期。**

这是 20 kHz 下的余量（2400 − 563）。做不到就**停下来报告**，不要自行降低环频——
降频要重新核对电流环极点，那是设计决策，不在本任务范围内。

---

## 2. 数值表示（照抄，不要改）

只有一个分数量（sin/cos 用 Q15），其余全部是**整数物理单位**。
这样不存在需要推导的标度因子，每个变量的量纲写在名字里。

| 量 | 单位 | 类型 | 范围 |
|---|---|---|---|
| 相电流 `ia/ib/ic`、`i_al/i_be`、`id/iq`、`iq_ref` | **ADC LSB**（1 LSB = 1.617 mA） | `int32_t` | ±2048 |
| 电压 `vd/vq`、积分器 | **µV** | `int32_t` | ±7×10⁶ |
| 电压（逆 Park 之后到 SVPWM） | **`VU` = 1024 µV** | `int32_t` | ±6800 |
| `sin`/`cos` | Q15（±32767） | `int16_t` | — |
| 角度 `theta` | 65536/电周期 | `uint16_t` | 直接用 `g_motor_angle.theta` |

**电流保持 LSB 而不是换成 mA**：那样省掉一次乘法和一次除法，
并且把 1.617 mA/LSB 折进增益里（第 3 节），数值范围也更小。

**积分器用 µV 而信号用 mV 量级**：这满足 roadmap 3.5 #1（积分器必须比信号路径宽）。
误差 1 LSB 时每拍增量 396 µV，**永远不会整除为 0**，所以不存在积分死区。

---

## 3. 常数（照抄，已算好）

```c
/* 20 kHz 控制环，Ts = 50 us。电流环带宽 1200 Hz（f_s/10 的上限内）。
 * 极点零点抵消：Kp = wc*Ld，Ki = wc*Rs，wc = 2*pi*1200 = 7540 rad/s
 *   Kp    = 7540 * 437.3e-6 H      = 3.297 V/A = 3297 uV/mA
 *   Ki*Ts = 7540 * 0.65 * 50e-6 s  = 0.2450 V/A = 245.1 uV/mA
 * 再乘 1.617 mA/LSB 折算到 LSB：
 */
#define FX_KP_UV_PER_LSB   5331   /* 3297 * 1.617 */
#define FX_KI_TS_UV_PER_LSB 396   /* 245.1 * 1.617 */

#define FX_INV_SQRT3_Q15   18919  /* 1/sqrt(3) * 32768 */
#define FX_SQRT3_2_Q15     28378  /* sqrt(3)/2 * 32768 */
#define FX_HALF_Q15        16384
```

`Rs = 0.65 Ω`、`Ld = 437.3 µH` 是实测值（相间 1.3 Ω / 874.7 µH 折半），
不是数据手册值。`1.617 mA/LSB` 也是实测（roadmap 1.2.1）。

---

## 4. 算法（逐步照抄，与 `sim/foc_controller_step.m` 的第 1/3/5/6/8 步对应）

`sin`/`cos` 表：256 项 `int16_t`，`s_sin_q15[k] = round(sin(2*pi*k/256) * 32767)`。
`cos` 用同一张表偏移 64 项：`cos_q15 = s_sin_q15[(idx + 64) & 255]`。
索引 `idx = theta >> 8`。**不要插值**——量化 1.4°，而 Hall 锚点本身的几何误差就有 ±9°
（roadmap 5.1），插值是在打磨错误的那一项。

### 4.1 三分流零序去除 + Clarke

```
ia = -(adc[IU] - 2048)        /* 分流为正 = 电流流出该相，故取负，见 roadmap 1.2.1 */
ib = -(adc[IV] - 2048)
ic = -(adc[IW] - 2048)

i0 = (ia + ib + ic) / 3       /* 星形接法无零序，这一项全是测量误差，实测 rms 约 3 LSB */
ia -= i0;  ib -= i0
i_al = ia
i_be = ((ia + 2*ib) * FX_INV_SQRT3_Q15) >> 15
```

`/3` 是**允许的唯一除法之外**的除法：改写成 `(x * 21846) >> 16`（21846 = 65536/3）。
M0 无硬件除法，见 roadmap 3.5 #2。

### 4.2 Park

```
idx  = theta >> 8
sn   = s_sin_q15[idx]
ct   = s_sin_q15[(idx + 64) & 255]
id   = ( i_al*ct + i_be*sn) >> 15
iq   = (-i_al*sn + i_be*ct) >> 15
```

中间乘积最大 2048 × 32767 × 2 = 1.3×10⁸，`int32_t` 装得下。

### 4.3 两个电流 PI

```
vdc_mv = (adc[VBUS] * 4058) >> 10      /* 3.9639 mV/LSB * 1024 = 4058 */
若 vdc_mv < 1000 则 vdc_mv = 1000      /* 台架上 ADC 未转换时保护 */
vmax_uv = vdc_mv * 577                 /* /sqrt(3)，577 = 1000/sqrt(3) */

ed = 0 - id                            /* SPMSM，无弱磁，id_ref = 0 */
eq = iq_ref - iq

id_int_uv = clamp(id_int_uv + FX_KI_TS_UV_PER_LSB * ed, -vmax_uv, vmax_uv)
iq_int_uv = clamp(iq_int_uv + FX_KI_TS_UV_PER_LSB * eq, -vmax_uv, vmax_uv)

vd_uv = FX_KP_UV_PER_LSB * ed + id_int_uv
vq_uv = FX_KP_UV_PER_LSB * eq + iq_int_uv
```

`4058` 的来历：分压比 4.9（`R30` 39k / `R31` 10k），VDDA 3.3124 V 满量程 4095 →
`3.3124/4095*4.9 = 3.9639 mV/LSB`，乘 1024 得 4058。

**每个累加点都要显式饱和**（roadmap 3.5 #3）：M0 没有 `SSAT`，溢出会符号翻转、
输出剧变，可能烧板。

**交叉解耦与反电动势前馈这一版不做**：`we*Lq*iq` 在 24 电周期/s 下只有约 0.1 V，
而 `we*λ` 那项需要 `λ`，它在 `sim/mc_params.m` 里是**估算值**而非实测，
阶段 F 测出 `Ke` 之后再加。浮点版的 `MOTOR_FOC_LAMBDA` 同样留零。

### 4.4 电压圆限幅

```
vd_u = vd_uv >> 10                     /* 转成 VU = 1024 uV */
vq_u = vq_uv >> 10
vmax_u = vmax_uv >> 10
vmag2 = vd_u*vd_u + vq_u*vq_u          /* 最大 2*6800^2 = 9.2e7，装得下 */
if (vmag2 > vmax_u*vmax_u) {
    vmag = isqrt32(vmag2)              /* 见下，无除法 */
    vd_u = (vd_u * vmax_u) / vmag      /* 除法，理由见下 */
    vq_u = (vq_u * vmax_u) / vmag
}
```

`isqrt32` 用标准的逐位二分整数平方根（无除法、无浮点）：

```c
static uint32_t isqrt32(uint32_t x)
{
  uint32_t r = 0U;
  uint32_t b = 1UL << 30;

  while (b > x) { b >>= 2; }
  while (b != 0U) {
    if (x >= (r + b)) { x -= r + b; r = (r >> 1) + b; }
    else              { r >>= 1; }
    b >>= 2;
  }
  return r;
}
```

**这两处除法是允许的**（roadmap 3.5 #2 要求解释清楚）：只在电压真正饱和的那些拍上执行，
而正常低电流工作点不饱和。若实测周期数超预算且饱和频繁，改用
`sc = (vmax_u<<15)/vmag` 复用一次除法，而不是两次。

### 4.5 逆 Park + SVPWM

```
v_al = (vd_u*ct - vq_u*sn) >> 15
v_be = (vd_u*sn + vq_u*ct) >> 15

va = v_al
vb = (-v_al*FX_HALF_Q15 + v_be*FX_SQRT3_2_Q15) >> 15
vc = (-v_al*FX_HALF_Q15 - v_be*FX_SQRT3_2_Q15) >> 15

vcom = (max(va,vb,vc) + min(va,vb,vc)) >> 1
```

### 4.6 输出到 TIM1

**直接算 `CCR`，不要先算 0..1 的 duty**（那会引入一次浮点或一次额外除法）：

```
k = (ARR * 33554) / vdc_mv             /* 每拍一次除法；33554 = 1.024 * 32768 */
ccr[j] = ARR/2 + (((v[j] - vcom) * k) >> 15)
ccr[j] = clamp(ccr[j], 0, ARR)
```

`33554` 的来历：`v` 的单位是 `VU` = 1.024 mV，`vdc_mv` 是 mV，
所以 `duty = (Δv * 1.024) / vdc_mv`，把 `1.024` 提到 Q15 就是 `1.024 * 32768 = 33554`。
自检：`ARR = 1199`、`vdc = 11980 mV` 时 `k = 3358`；SVPWM 注入共模后
`|v - vcom|` 上限是 `vdc/2 = 5850 VU`，代入得偏移 599 = `ARR/2`，正好占满半个周期，**量纲对**。

`ARR` 用 `__HAL_TIM_GET_AUTORELOAD(&htim1)`，当前是 1199。

然后调用现有的死区补偿输出函数——**不要自己写 CCR**：

```c
MotorPwm_SetDutiesCcr(ccr[0], ccr[1], ccr[2], ia, ib, ic);
```

这个函数**还不存在**，你要在 `motor_pwm.c` 里加，参照现有的
`MotorPwm_SetDuties()`：同样的死区补偿（按每相电流符号加减 `DTG/2` = 36 计数，
±0.05 A 内线性过渡防噪声抖动），但输入是 `CCR` 计数而不是浮点 duty，
且电流参数改成 `int32_t` 的 LSB。**保留那段注释解释为什么死区误差不是共模**。

---

## 5. 验收（三条全部要过，缺一条就是没完成）

### 5.1 编译与静态检查

```bash
export PATH="/Applications/CMake.app/Contents/bin:$PATH"
cmake -S src/firmware/stm32f030 -B src/firmware/stm32f030/build/Release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DMOTOR_AUTO_START=OFF
cmake --build src/firmware/stm32f030/build/Release
```

- 零 error、零新增 warning。
- **`MotorFocFx_Step` 里不得出现任何 `__aeabi_f*`、`__aeabi_d*`**。用这条命令验证：

```bash
OD=/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-objdump
$OD -d src/firmware/stm32f030/build/Release/motor-ctrl.elf \
   --disassemble=MotorFocFx_Step | rg -o '__aeabi_[a-z0-9]+' | sort | uniq -c
```

只允许出现 `__aeabi_uidiv` / `__aeabi_idiv`（第 4.4、4.6 节说明过的那几处）。
**出现任何浮点符号就是没达标**——那正是整个任务要消除的东西。

### 5.2 周期数

```bash
./src/firmware/stm32f030/scripts/ctrl_bench.sh
```

你要在 `App/ctrl_bench.c` 里加一个案例测 `MotorFocFx_Step`（照抄现有
`bench_foc_step` 的写法，把 `MOTOR_CTRL_BENCH_CASES` 加 1，并在
`scripts/ctrl_bench.sh` 的 `labels`、`read_words` 字数、切片下标里同步加一项）。

> **判据：中位数 < 1837 周期。**

电机全程不通电，这一步没有任何安全风险。

### 5.3 与浮点版的数值对照

这是抓标度错误的唯一可靠手段，**必须做**。在 `ctrl_bench.c` 里加一个对照函数
（不是计时案例，是一个把结果写进 `g_ctrl_bench` 之外的独立结构的检查）：

对每组测试输入：

1. 写 `g_motor_adc_raw[0..3]` 和 `g_motor_angle.theta`（都是 `volatile`，可直接写）
2. `MotorFoc_SetMode(MOTOR_FOC_OBSERVE)` 后调 `MotorFoc_Step()`，读 `g_motor_foc.id_ma/iq_ma`
3. 同样输入下调 `MotorFocFx_Step()`（OBSERVE 模式），读定点版的 `id/iq`
4. 把定点的 LSB 换成 mA（`× 1617 / 1000`）后比较

测试输入至少覆盖这 8 组（`theta` 取 0 / 8192 / 16384 / 40000 各配两种电流）：

| adc[IU] | adc[IV] | adc[IW] | adc[VBUS] | theta |
|---|---|---|---|---|
| 2048 | 2048 | 2048 | 3024 | 0 |
| 2814 | 1282 | 2048 | 3024 | 0 |
| 2814 | 1282 | 2048 | 3024 | 8192 |
| 2814 | 1282 | 2048 | 3024 | 16384 |
| 2814 | 1282 | 2048 | 3024 | 40000 |
| 1282 | 2814 | 2048 | 3024 | 8192 |
| 2300 | 2300 | 1544 | 3024 | 24576 |
| 2048 | 2814 | 1282 | 3024 | 49152 |

> **判据：每组 `|Δid| ≤ 4 mA` 且 `|Δiq| ≤ 4 mA`。**

4 mA 是约 2.5 个 ADC LSB，即定点截断与浮点舍入的合理差异。
**差得多说明标度或符号错了**，逐级打印中间量（`i_al`、`i_be`、`sn`、`ct`）定位，
不要靠调参掩盖。

---

## 6. 明确不属于本任务

**做到 5.1~5.3 就交付，不要继续往下做。** 下面每一项都需要通电并解读带歧义的数据，
或者会加大电流，必须由具备判断能力的人接手：

| 不要做 | 原因 |
|---|---|
| 闭合电流环、让电机由 FOC 驱动 | 角度对齐尚未标定，闭环会把电流全力打进 d 轴 |
| 标定 `MOTOR_ANGLE_ANCHOR_TRIM` | 要用 `id` 的周期-6 模式反解，属解读性工作，见 roadmap 5.1 |
| 启用 TIM1 BKIN、改过流保护 | 历史上启用 BKIN 曾导致 MOE 锁死 |
| 提高 duty 或 `MOTOR_FOC_IMAX` | 见 measurement-validity 1.6 与 roadmap 0.2 |
| 改 `sim/codegen_stm32/` 下任何文件 | 生成物，roadmap 9.5 |
| 为了让判据通过而调 5.2/5.3 的阈值 | 阈值是从预算和分辨率推出来的，不是可调项 |

## 7. 收尾（每次通电测试后都要做）

```bash
source src/firmware/stm32f030/scripts/bench_common.sh
motor_off
cmake -S src/firmware/stm32f030 -B src/firmware/stm32f030/build/Release \
  -DCMAKE_BUILD_TYPE=Release -DMOTOR_AUTO_START=OFF
cmake --build src/firmware/stm32f030/build/Release && flash_elf
./src/firmware/stm32f030/scripts/board_check.sh      # 必须 PASS
```

`board_check.sh` 不 PASS 就不算收尾完成。它已经两次抓到被遗留的通电状态。

## 8. 交付时报告什么

按 roadmap 9.3 的模板：

```
阶段：E（定点 FOC 实现）
固件参数：<完整 cmake -D 列表>
5.1 反汇编输出：<__aeabi_* 计数>
5.2 周期数：中位数 __ / 最大 __，预算 1837
5.3 对照表：8 组的 Δid / Δiq
遗留问题：<列表>
```
