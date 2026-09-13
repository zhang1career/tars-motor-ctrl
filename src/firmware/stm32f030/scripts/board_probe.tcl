# Probe the motor-ctrl mainboard v0.1 analog front end and fault lines.
#
# Deliberately does NOT halt the CPU: every access goes through the DAP while
# firmware keeps running.
#
# Idle: this script takes over ADC1 to read NTC / VREFINT / V_BOOST. That is
# safe only when the motor is off -- the control tick is now DMA TC on ADC1
# (roadmap 4.1), so stealing the ADC while spinning stops commutation.
# --running must set probe_skip_adc 1 and read currents from g_motor_adc_raw.
#
# Emits machine-parsable "KEY value..." lines for scripts/board_check.sh.

if {![info exists probe_passes]} {
    set probe_passes 3
}
if {![info exists probe_skip_adc]} {
    set probe_skip_adc 0
}

proc poll {addr mask want {n 2000}} {
    for {set i 0} {$i < $n} {incr i} {
        set v [mrw $addr]
        if {($v & $mask) == $want} {
            return $v
        }
    }
    echo [format "TIMEOUT 0x%08x got=0x%08x mask=0x%x want=0x%x" \
              $addr [mrw $addr] $mask $want]
    return -1
}

# CR1 CCER BDTR SR ARR RCR. ARR and RCR fix the control-loop rate outright:
# centre-aligned counting gives an update at both overflow and underflow, so
# RCR=1 is what makes the ISR run at 20 kHz instead of 40 kHz.
echo [format "TIM1 %u %u %u %u %u %u" \
          [mrw 0x40012C00] [mrw 0x40012C20] [mrw 0x40012C44] [mrw 0x40012C10] \
          [mrw 0x40012C2C] [mrw 0x40012C30]]

set moder_a [mrw 0x48000000]
echo [format "GPIOA %u %u" $moder_a [mrw 0x48000010]]
echo [format "GPIOB %u" [mrw 0x48000410]]

if {$probe_skip_adc} {
    echo "SKIPADC 1"
    return
}

# PA0..PA5 to analog input
mww 0x48000000 [expr {$moder_a | 0x00000FFF}]

# reset ADC1, clock it, start the dedicated 14 MHz ADC clock
mmw 0x4002100C 0x00000200 0
mmw 0x4002100C 0 0x00000200
mmw 0x40021018 0x00000200 0
mmw 0x40021034 0x00000001 0
poll 0x40021034 0x2 0x2

if {[mrw 0x40012408] & 1} {
    mmw 0x40012408 0x00000002 0
    poll 0x40012408 0x1 0
}

mww 0x40012408 0x80000000
poll 0x40012408 0x80000000 0

mww 0x40012400 0x0000001F
mww 0x40012408 0x00000001
poll 0x40012400 0x1 0x1

mww 0x4001240C 0x00000000
mww 0x40012410 0x00000000
mww 0x40012414 0x00000007
mww 0x40012708 0x00C00000
sleep 5

for {set pass 0} {$pass < $probe_passes} {incr pass} {
    foreach ch {0 1 2 3 4 5 16 17} {
        mww 0x40012428 [expr {1 << $ch}]
        mww 0x40012400 0x0000001F
        mmw 0x40012408 0x00000004 0
        poll 0x40012400 0x4 0x4
        echo [format "ADC %d %d" $ch [mrw 0x40012440]]
    }
}

# Factory calibration: 0x1FFFF7B8 = TS_CAL1 (30 C), 0x1FFFF7BA = VREFINT_CAL,
# 0x1FFFF7C2 = TS_CAL2 (110 C). All taken at VDDA = 3.3 V.
set cal0 [mrw 0x1FFFF7B8]
set cal2 [mrw 0x1FFFF7C0]
echo [format "CAL %d %d %d" \
          [expr {$cal0 & 0xFFFF}] [expr {($cal0 >> 16) & 0xFFFF}] \
          [expr {($cal2 >> 16) & 0xFFFF}]]

mmw 0x40012408 0x00000002 0
mww 0x48000000 $moder_a
echo [format "RESTORED %u" [mrw 0x48000000]]
