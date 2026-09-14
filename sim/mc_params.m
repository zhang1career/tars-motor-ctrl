function p = mc_params()
%MC_PARAMS Single source of truth for motor, control and simulation params.
%   Returns a struct used by the plant, the FOC controller and the build
%   scripts.
%
%   TARGET CHANGED: this model was written for the STM32F429 (Cortex-M4F,
%   single-precision FPU).  The current target is the STM32F030K6 --
%   Cortex-M0, NO FPU, no hardware divide.  Every float op becomes a libgcc
%   call, so the generated single-precision controller does not meet the
%   20 kHz budget (2400 cycles/tick).  See docs/roadmap.md section 3 for the
%   fixed-point decision gate and the capability-boundary analysis.
%
%   PARAMETER STATUS: values marked [MEASURED] come from bench work on the
%   real hardware; values marked [TODO] are still the original estimates for
%   a generic 24 V / 60 W / 3000 rpm PMSM and are NOT this motor.  Do not
%   base controller gains or fixed-point scaling on a [TODO] value.
%
%   REGENERATION REQUIRED: codegen_stm32/foc_step_stm32.c has these values
%   baked in as literals (e.g. Kp_i, Ld, Lambda).  Editing this file has no
%   effect on the firmware until gen_code.m is re-run under MATLAB Coder.
%
%   IMPORTANT (codegen): all values are computed into local variables and
%   the struct is assembled at the end, because MATLAB Coder forbids adding
%   fields to a struct after it has been read.
%#codegen

% ----------------------------------------------------------------------
% MOTOR
% ----------------------------------------------------------------------
% [MEASURED] pole pairs.  Hall6 low-duty run, 10 s window: 311 hall edges
% -> 51.8 electrical revs; shaft counted 13–14 mechanical revs ->
% 51.8 / 13.5 = 3.8, confirms 4.
PolePairs = 4;            % pole pairs                          [-]

% [MEASURED] phase-to-phase resistance is 1.3 Ohm on all three pairs, so the
% per-phase value is half of that.  Keep R and L in the same convention.
Rs        = 0.65;         % stator resistance / phase           [Ohm]

% [MEASURED] LCR meter at 1 kHz, phase to phase: 860 / 881 / 883 uH
% (yellow-green / yellow-blue / green-blue).  Mean 874.7 uH -> 437.3 uH per
% phase.  The 2.6% spread across the three pairs supports the SPMSM Ld = Lq
% assumption.  Consequence: tau = L/Rs = 673 us, current-loop plant pole
% f_pole = 236 Hz, so Fpwm >= 2.4 kHz suffices -- the present 20 kHz is 85x
% the pole and fixed-point conversion is NOT required for timing reasons.
% CAVEAT: this is a 1 kHz small-signal value.  Incremental inductance at the
% operating current is lower (saturation); if it drops below ~220 uH per phase
% the pole moves past 470 Hz and a 5 kHz loop becomes marginal.  Re-check from
% a current-rise slope (di/dt = V/L) once phase C current sensing exists.
Ld        = 437.3e-6;     % d-axis inductance                   [H]
Lq        = 437.3e-6;     % q-axis inductance (SPMSM: Ld=Lq)    [H]

% [MEASURED] 2026-09-11 FOC CURRENT dir0, 520P at 0%, vq not saturated:
%   vq=2.10 V, iq=301 mA, 256 ms foc_ang wrap = 47.0 elec/s (72 hall edges).
%   λ = (vq − Rs·iq)/ω_e = (2.10 − 0.65·0.301)/(47·2π) = 6.45 mWb.
%   Trace-mean vq=2.045 V gives 6.26 mWb.  Use 6.3 mWb.
%   2026-09-14: two independent checks agree to ~2% —
%     bus power at tick 5 / 80 elec/s, and Kt·Δiq vs tick4→5 ΔP_mech.
%   Do not use DAP edge/s without wall-clock dt (old 71–92 elec/s were
%   openocd overhead).  Does not change Kp/Ki (those come from Ld/Rs).
Lambda    = 0.0063;       % PM flux linkage (peak)              [Wb]

% [TODO] plant inertia/friction for the MATLAB speed PI only.
% The on-target loop (motor_foc_fx.c FX_W_*) does not use J or B.
J         = 1.5e-5;       % rotor inertia                       [kg*m^2]
B         = 1.0e-5;       % viscous friction                    [N*m*s]

RatedPower = 60;          % [TODO] unknown for this motor       [W]
RatedSpeed = 3000;        % [TODO] unknown for this motor       [rpm]

% [MEASURED] bench bus voltage.
Vdc        = 12;          % DC bus                              [V]

% [MEASURED-BOUND] bench-safe cap, NOT a motor rating.  Constraints:
%   - supply is current limited at about 1.6 A on the bus
%   - TIM1 BKIN from nFAULT is ON (MOTOR_PWM_BKIN); window is ±2.5 A
%   - board 3 has a thermal incident in its history
%   - current feedback is 1.617 mA/LSB (INA240A2)
Imax       = 3.0;         % peak phase current limit            [A]

% ----------------------------------------------------------------------
% SAMPLING / TIMING
% ----------------------------------------------------------------------
% [MEASURED] firmware runs TIM1 at 20 kHz centre-aligned with RepetitionCounter
% = 1, giving one control tick per PWM period (verified 19.95 kHz on target).
% Dead time is DTG=72 -> 1.5 us, which subtracts from the high-side pulse and
% therefore caps the usable modulation range.
Fpwm   = 20000;           % PWM / current-loop frequency        [Hz]
Ts     = 1/Fpwm;          % current-loop sample time            [s]
Nspeed = 10;              % speed loop runs every Nspeed steps (2 kHz)
Tss    = Nspeed*Ts;       % speed-loop sample time              [s]

% ----------------------------------------------------------------------
% CURRENT CONTROLLERS (pole-zero cancellation tuning)
%   Plant 1/(Ls+Rs); PI with Ki/Kp = Rs/Ls cancels the motor pole.
%
%   COUPLED TO THE SAMPLE RATE: usable bandwidth is about Fpwm/10.  The 1200 Hz
%   below assumes the 20 kHz loop.  If the loop is dropped to 5 kHz to fit the
%   float controller (roadmap 3.3), the ceiling becomes 500 Hz and wc_i must
%   come down with it -- e.g. 2*pi*400 gives Kp_i = 1.10, Ki_i = 1634.
%   Leaving wc_i at 1200 Hz while sampling at 5 kHz is unstable.
% ----------------------------------------------------------------------
wc_i = 2*pi*1200;         % current loop bandwidth              [rad/s]
Kp_i = wc_i*Ld;           % proportional gain (V/A)
Ki_i = wc_i*Rs;           % integral gain (V/(A*s))

% ----------------------------------------------------------------------
% SPEED CONTROLLER (MATLAB / sim only — ~1 decade below current loop)
%   On-target scores (2026-09-14, wrap = foc_ang net/dt, not w_meas):
%     hold 80 / 130 elec/s both dirs; 80→130 10–90% ~300 ms; overshoot
%     a few elec/s.  12 V base speed ~150 elec/s wrap.  No FW region
%     (λ/Ld = 14.4 A, Imax = 3 A).  See docs/report/speed-loop-20260914.md.
% ----------------------------------------------------------------------
Kt   = 1.5*PolePairs*Lambda;  % 0.0378 N*m/A [MEASURED via λ]
wc_w = 2*pi*25;               % speed loop bandwidth [rad/s]
Kp_w = J*wc_w/Kt;             % (A/(rad/s))
Ki_w = Kp_w*wc_w/8;           % (A/(rad/s)/s)

% ----------------------------------------------------------------------
% SENSORLESS: flux observer + PLL (tracking loop)
%   ARCHITECTURE MISMATCH: the hardware has hall sensors on PB3/PB4/PB5 and
%   the firmware plan (roadmap phase D) derives the angle from hall edges plus
%   speed extrapolation.  The generated controller already exposes the override
%   hooks (g_tars_foc_hall_en / _theta / _w_est, foc_step_stm32_hall_bootstrap).
%   These sensorless gains are therefore unused on this target unless the
%   sensorless path is deliberately revived.
% ----------------------------------------------------------------------
Whp     = 2*pi*3;             % flux-integrator HP cutoff [rad/s]
wn_pll  = 2*pi*40;            % PLL natural frequency [rad/s]
zeta    = 1.0;
Kp_pll  = 2*zeta*wn_pll;
Ki_pll  = wn_pll^2;
Wspd_lp = 2*pi*60;            % speed estimate LP filter [rad/s]

% ----------------------------------------------------------------------
% OPEN-LOOP I/F STARTUP
% ----------------------------------------------------------------------
Istart       = 1.0;           % forced current magnitude [A]
SpeedHandoff = 300;           % handoff speed [rpm]
Talign       = 0.10;          % rotor alignment time [s]
Tramp        = 0.80;          % ramp time to handoff [s]

Vmax = Vdc/sqrt(3);           % max phase voltage amplitude (SVPWM) [V]

% ----------------------------------------------------------------------
% assemble struct (no reads between additions -> codegen safe)
% ----------------------------------------------------------------------
p = struct( ...
    'PolePairs', PolePairs, 'Rs', Rs, 'Ld', Ld, 'Lq', Lq, 'Lambda', Lambda, ...
    'J', J, 'B', B, 'RatedPower', RatedPower, 'RatedSpeed', RatedSpeed, ...
    'Vdc', Vdc, 'Imax', Imax, 'Fpwm', Fpwm, 'Ts', Ts, 'Nspeed', Nspeed, ...
    'Tss', Tss, 'Kp_i', Kp_i, 'Ki_i', Ki_i, 'Kt', Kt, 'Kp_w', Kp_w, ...
    'Ki_w', Ki_w, 'Whp', Whp, 'Kp_pll', Kp_pll, 'Ki_pll', Ki_pll, ...
    'Wspd_lp', Wspd_lp, 'Istart', Istart, 'SpeedHandoff', SpeedHandoff, ...
    'Talign', Talign, 'Tramp', Tramp, 'Vmax', Vmax );
end
