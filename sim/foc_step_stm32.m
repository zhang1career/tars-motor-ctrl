function [duty_a, duty_b, duty_c, theta_est, speed_est_rpm, id, iq] = ...
        foc_step_stm32(ia, ib, ic, vdc, speed_ref_rpm, enable) %#codegen
%FOC_STEP_STM32 Embeddable single-precision FOC step for STM32F429.
%
%   This is the CODE-GENERATION ENTRY POINT for the portable controller.
%   Call it once per PWM period (20 kHz) from the ADC end-of-conversion /
%   PWM-update ISR on the STM32F429.  All I/O is single precision (real32_T)
%   so the Cortex-M4F FPU is used directly.
%
%   Inputs:
%     ia, ib, ic     measured phase currents [A] (single), three shunts
%     vdc            measured DC-bus voltage [V] (single)
%     speed_ref_rpm  speed command [rpm] (single)
%     enable         1.0 run / 0.0 disable (single)
%
%   Outputs:
%     duty_a/b/c     PWM duty cycles in [0,1] -> load into TIM1 CCRx
%     theta_est      estimated electrical angle [rad]
%     speed_est_rpm  estimated mechanical speed [rpm]
%     id, iq         measured dq currents [A]
%
%   Controller state and parameters are held in static (persistent) storage
%   and initialized on the first call.  mc_params() returns compile-time
%   constants, so Embedded Coder folds it into constants (no run-time
%   double math on the MCU).

persistent st initialized
p = coder.const(mc_params());        % folded to compile-time constants
if isempty(initialized)
    st = foc_state_init(p);
    initialized = true;
end

[duty, dbg, st] = foc_controller_step(ia, ib, ic, vdc, speed_ref_rpm, enable, st, p);

duty_a = duty(1);
duty_b = duty(2);
duty_c = duty(3);
theta_est     = dbg(1);
speed_est_rpm = dbg(2);
id            = dbg(3);
iq            = dbg(4);
end
