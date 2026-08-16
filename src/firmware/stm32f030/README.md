# motor-ctrl 固件（STM32F030K6）

Hall 闭环 6-step BLDC 控制，自 TARS `tars_hall6.c` 迁移。纯开环（`tars_openloop.c` 一路）保留作台架诊断。

## 目录

```
src/firmware/stm32f030/
├── App/motor_app.c       # 默认参数 + start/stop 封装
├── Core/                 # main、MSP、中断
├── board_pins.h          # PCB 管脚
├── CMakeLists.txt
├── Drivers/              # STM32CubeF0 HAL（自 tars-io-mux 复制）
├── scripts/              # flash.sh + 台架扫描脚本（见下）
└── openocd.cfg

src/platforms/stm32f030/  # hall6 / 开环算法 + PWM / Hall / TIM1 控制环
```

## 默认运行参数

台架实测得出，**不等于 TARS 的数字**（死区口径不同，见「已知陷阱」）：

```
hall6 闭环：phase 3, kick duty 20%, run duty 20%, direction 0（本接线下为逆时针）
TIM1：      20 kHz 中心对齐, DTG=72（1.5 us 死区）, 控制环 20 kHz
```

实测：绕组约 1.6 A，转起来后反电动势把母线压到约 70 mA、约 24 电周期/s。

## 构建

```bash
cmake -S src/firmware/stm32f030 -B src/firmware/stm32f030/build/Release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release
cmake --build src/firmware/stm32f030/build/Release
```

产物：`build/Release/motor-ctrl.elf`（约 11.1 KB Flash / 1.5 KB RAM）。

上电自动启动 hall6 闭环（调试用，**会立即让电机转起来**）：

```bash
cmake -S src/firmware/stm32f030 -B src/firmware/stm32f030/build/Release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DMOTOR_AUTO_START=ON -DMOTOR_START_HALL6=ON
```

改参数时优先用 cmake 选项，不要改源码默认值：

| 选项 | 作用 |
|------|------|
| `MOTOR_AUTO_START` | 上电即启动（默认 OFF） |
| `MOTOR_START_HALL6` | 自启动走 hall6 闭环，否则走纯开环（默认 OFF） |
| `MOTOR_HALL6_KICK_DUTY` / `MOTOR_HALL6_RUN_DUTY` | hall6 起转 / 运行 duty（上限 25%） |
| `MOTOR_HALL6_PHASE` / `MOTOR_HALL6_CCW` | hall6 换相相位偏移 0..5 / 反转 |
| `MOTOR_TIM1_DTG` | TIM1 BDTR 死区寄存器原始值（默认 72） |
| `MOTOR_DUTY_PCT` / `MOTOR_PULSE_COUNTS` | 开环 duty 百分比 / 直接给 CCR 计数 |
| `MOTOR_STEP_MS` / `MOTOR_PHASE_OFFSET` | 开环换相周期（0 = 静态保持不换相）/ 相位偏移 |
| `MOTOR_HS_ONLY` | 诊断：只驱动高边，不开同臂低边 |

**注意**：这些是 cmake cache 变量，复用旧 build 目录会残留上次的值。切换参数组时先 `rm -rf build/Release`，或显式传全部相关选项。

## 烧录

```bash
./src/firmware/stm32f030/scripts/flash.sh
```

## 台架脚本

都会在收尾时刷回 `AUTO_START=OFF` 的安全固件，并在探头缺失或烧录失败时报错退出。

| 脚本 | 用途 |
|------|------|
| `scripts/bench_common.sh` | 公共函数（探头预检、烧录校验、断电、DMM 读数重试），被下面三个 source |
| `scripts/duty_sweep.sh` | 按 duty 百分比扫描，读母线电流 |
| `scripts/ccr_sweep.sh` | 按 CCR 计数扫描（duty 百分比在死区阈值附近粒度不够时用） |
| `scripts/step_table.sh` | 逐个静态保持 6 个换相步，检查三相是否对称 |

## API

| 函数 | 说明 |
|------|------|
| `MotorApp_Init()` | 初始化外设 + 应用默认参数 |
| `MotorApp_Start()` | 启动 hall6 闭环（kick → Hall 换相） |
| `MotorApp_Stop()` | 停转（关 MOE 与全部输出） |
| `MotorApp_StartOpenloop6()` | 纯开环 6-step，用固件默认参数 |
| `MotorApp_StartOpenloop6Cfg(phase, uv_perm, dir, duty, step_ms)` | 纯开环，显式参数 |
| `MotorApp_DiagGPhase(duty)` | 诊断：固定单步通电 |
| `MotorHall6_*` / `MotorOpenloop_*` | 完整 API（`src/platforms/stm32f030/*.h`） |

`motor_swd_entry[]` 导出上述入口地址，便于 SWD 直接调用。

## 已知陷阱

这几条都是台架上花了很大代价才定位的，改动 PWM 相关代码前务必先读：

1. **低边常通不能用 `CCR=0`。** `CCxE=0, CCxNE=1` 时 OCxN 引脚**直接跟随 OCxREF**（不反相、不插死区），所以 `CCR=0` 会让低边恒关、回流臂断开、电机毫无力矩。必须让 OCxREF 恒为有效，即 `CCR = ARR+1`。TARS 在 F429 上是同样写法。
2. **中心对齐必须 `RCR=1`。** 上溢和下溢各产生一次 update 事件，`RCR=0` 时控制环会跑在 `2 × MOTOR_PWM_HZ`，所有基于 `MOTOR_CTRL_ISR_HZ` 的时间常数（Hall 消抖、堵转超时、kick 间隔）都会差一倍。
3. **DTG 是非线性编码，且死区要从 HS 脉冲里减掉。** `DTG[7:5]=0xx` 才是线性；`DTG=144` 实际是 3.33 us 而非 3 us。有效导通时间 = `2×CCR − DT`，所以「duty 百分比相同」不代表驱动量相同——跨 MCU 移植要对齐**有效导通时间**，不是 duty 数字。
4. **`MotorOpenloop_SetStepMs()` 把非零值限幅到 5..500 ms**，传 60000 会被压成 500 ms。需要真正不换相时传 **0**。
5. **半桥板 VOUT 对地的滤波电容必须拆掉。** 那块板按 buck 输出级设计，VOUT 上的电容会被高边充、低边放，形成一条约 96 mΩ 的通路吃掉约 1.5 A（∝ 导通时间、与死区无关、与电机是否接入无关），电机只能吃残羹。拆掉后母线电流降约 80~100 倍，并从「线性 ∝ 导通时间」变为「平方 ∝ duty²」的正常电机特性。
6. **`MotorApp_Start()` 等 SWD 调用不可靠**（gdb `call` 返回正常但外设状态常对不上），台架验证请用 `MOTOR_AUTO_START` 烧录后 reset 的路径。另外 OpenOCD `program ... reset` 会在 `Reset_Handler` 留断点，测量前必须 `rbp all` + `resume`，否则 CPU 根本没跑到 `main`。

I²C 从机 / TNB 寄存器：待 `shared/tnb` 与 `App/motor_node.c` 后续添加。
