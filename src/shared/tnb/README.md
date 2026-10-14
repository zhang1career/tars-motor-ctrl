# shared/tnb — motor-ctrl 从机协议

与 TARS 主控 `App/node_bus/tnb_protocol.h` 对齐。产品号 `0x0005`，默认板号 16 → 地址 `0x50`。

不参与 ARP（地址固定，不靠 SMBus 动态分配）。能力表只有 MOTOR + ALERT。转速成绩用 `0xAE` 展开电角度 Q16：`wrap = Δacc / 65536 / dt`。

写带可选 PEC（SMBus CRC-8，多项式 0x07）：末字节对得上就剥掉。读固定「1 字节数据 + PEC」（从机不知道主机还要读几字节）。TARS 认出 `0x0005` 之后收发都带 PEC。TEMP / VDDA / 板 NTC / V_BOOST 约 0.5 s 更新一次（短暂停一下电流 ADC）。母线低于 8 V 时不锁 nFAULT / 过温。运行中换向是速度环过零，不是停桥再 hall6。

遥测 `0xB4`–`0xC3`（和 `0xA0`–`0xB3` 连着读）：

| 寄存器 | 量 | 备注 |
|---|---|---|
| `0xB4` VBOOST | 电荷泵 mV | PA5 |
| `0xB6` T_BOARD | 主板 RT1，0.1 °C | PA0。`0x8000` = 无效 |
| `0xB8` T_CASE | J15 电机壳，0.1 °C | **v0.1 没有 ADC**，恒 `0x8000`。过温只看 `sense` 的 nOTEMP / `flt` |
| `0xBA` T_MCU | 片内结温 | 与 `0x26` 同一数 |
| `0xBC` CLOCK | kHz | 低于约 40000 不让转 |
| `0xBE` SENSE | 引脚与品质位 | nFAULT、nOTEMP、时钟、换向、电压顶 |
| `0xC0`/`0xC2` vd/vq | mV | 电压指令，看有没有顶满 |
