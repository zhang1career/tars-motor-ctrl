# src/platforms/stm32f030 — STM32F030 平台层

已从 TARS 迁移的 Hall 6-step 电机控制：

| 文件 | 职责 |
|------|------|
| `hall6.c/.h` | Hall 闭环 6-step：kick 启动、MOE 延迟释放、phase 偏移 |
| `openloop.c/.h` | 6-step / 3-step 开环 + Hall lock/kick/stall（实机诊断用） |
| `motor_pwm.c/.h` | TIM1 六路互补 PWM（20 kHz 中心对齐）、死区、MOE 控制 |
| `motor_hall.c/.h` | Hall GPIO 读取 |
| `motor_tick.c/.h` | 控制环 ISR 分发（hall6 / openloop），跑在 **TIM1 update** 上 |

正式控制方式是 **hall6 闭环**；`openloop.c` 只在实机排查时用（配合
`src/firmware/stm32f030/scripts/` 下的扫描脚本）。

默认应用参数见 `App/motor_app.c`，由实测得出：

```
phase 3, kick duty 20%, run duty 20%, direction 0（顺时针；1 为逆时针）
```

## 控制环时基

`MotorTick_Init()` 不配置 TIM3；控制环由 **TIM1 update 中断**驱动
（`MotorTick_OnTim1Update()`），与 TARS 一致。中心对齐计数在上溢和下溢各产生一次
update，因此 `motor_pwm.c` 里 `RepetitionCounter = 1`，才能得到
`MOTOR_CTRL_ISR_HZ`（20 kHz）而不是它的两倍。

`HAL_TIM_PeriodElapsedCallback()` 里的 TIM3 分支是迁移期遗留，当前 TIM3 未初始化。

## 相位状态编码

`motor_phase_mode_t` 三态含义与写入 TIM1 的方式：

| 状态 | CCER | CCR |
|------|------|-----|
| `MOTOR_PHASE_OFF` | 清 `CCxE` 与 `CCxNE` | 0 |
| `MOTOR_PHASE_PWM` | 置 `CCxE` 与 `CCxNE`（互补 + 死区） | 目标脉宽 |
| `MOTOR_PHASE_LOW` | 清 `CCxE`、置 `CCxNE` | **`ARR + 1`** |

`MOTOR_PHASE_LOW` 用 `ARR+1` 而不是 0：`CCxE=0, CCxNE=1` 时 OCxN 引脚直接跟随
OCxREF（不反相、不插死区），`CCR=0` 会让低边恒关、回流臂断开。

## 换向

`hall6_lookup(hall, reverse)` 与 `ol_lookup_step(hall, ccw)` 在反向时把 **PWM 相与
LOW 相互换**——同一转子位置下绕组电流反向，力矩反向而幅值不变，所以正反转效率对称。
仅靠 `direction` 位切换 hall 序列是不够的：`s_seq_ccw` 是 `s_seq_cw` 的反序，等价于
偏移 `−p`，而 `−p ≡ p (mod 6)` 恰好落在 phase 0 和 3 这两个唯一高效的偏移上。其余陷阱见
[`src/firmware/stm32f030/README.md`](../../firmware/stm32f030/README.md#已知陷阱)。

板级管脚见 `src/firmware/stm32f030/board_pins.h`。
