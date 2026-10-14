function [duty, dbg, st] = foc_controller_step(ia, ib, ic, vdc, speed_ref_rpm, enable, st, p) %#codegen
%FOC_CONTROLLER_STEP One control-tick of sensorless field-oriented control.
%
%   Sensorless speed-closed-loop FOC for a surface-mount PMSM, designed for
%   single-precision execution on the STM32F429 Cortex-M4F FPU.
%
%   Inputs (all scalar):
%     ia, ib, ic     measured phase currents (A), three low-side shunts
%     vdc            measured DC-bus voltage (V)
%     speed_ref_rpm  speed command (mechanical rpm)
%     enable         1 = run, 0 = disable (outputs 50% duty, holds state)
%     st             controller state struct (see foc_state_init)
%     p              parameter struct (see mc_params)
%
%   Outputs:
%     duty  1x3 single, three-phase PWM duty cycles in [0,1]
%     dbg   1x6 single, [theta_est, speed_est_rpm, id, iq, vd, vq]
%     st    updated state struct
%
%   Three-shunt sensing: remove zero-sequence (ia+ib+ic != 0 from offset /
%   gain mismatch) before Clarke.  Star-connected motor has no zero-sequence
%   current, so i0 = (ia+ib+ic)/3 is measurement error only.

% ---- constants (single) ----------------------------------------------
Ts   = single(p.Ts);
Rs   = single(p.Rs);
Ld   = single(p.Ld);
Lq   = single(p.Lq);
P    = single(p.PolePairs);
Vmax = single(vdc)/single(sqrt(3));      % SVPWM linear range, bus-tracking
SQ3  = single(sqrt(3));
TWO_PI = single(2*pi);
THIRD = single(1)/single(3);

% ---- 1. three-shunt: zero-sequence removal + Clarke ------------------
i0   = (single(ia) + single(ib) + single(ic)) * THIRD;
ia_c = single(ia) - i0;
ib_c = single(ib) - i0;
i_al = ia_c;
i_be = (ia_c + single(2)*ib_c) / SQ3;

% ---- 2. choose electrical angle --------------------------------------
if st.mode == uint8(0)
    theta = st.if_th;          % open-loop forced angle
else
    theta = st.pll_th;         % sensorless estimated angle
end
ct = cos(theta);
sn = sin(theta);

% ---- 3. Park transform -----------------------------------------------
id =  i_al*ct + i_be*sn;
iq = -i_al*sn + i_be*ct;

% ---- 4. reference generation -----------------------------------------
we_est = st.w_est;                              % electrical speed estimate
if st.mode == uint8(0)
    % ===== OPEN-LOOP STARTUP =====
    st.tmr  = st.tmr + Ts;
    dir     = sign(single(speed_ref_rpm));
    if dir == single(0), dir = single(1); end
    w_hand  = dir * TWO_PI*P*single(p.SpeedHandoff)/single(60);   % handoff elec speed
    if st.tmr < single(p.Talign)
        % --- phase 1: align rotor to theta=0 with d-axis current ---
        st.if_w  = single(0);
        st.if_th = single(0);
        id_ref = single(p.Istart);
        iq_ref = single(0);
    else
        % --- phase 2: ramp frequency with d-axis (stepper-like) forcing ---
        %   Current vector on the forced d-axis; the rotor follows it with a
        %   load angle (well damped).  Torque comes from the natural lag.
        dw = w_hand/single(p.Tramp);            % accel (elec rad/s^2)
        st.if_w  = st.if_w + dw*Ts;
        if dir > single(0)
            if st.if_w > w_hand, st.if_w = w_hand; end
        else
            if st.if_w < w_hand, st.if_w = w_hand; end
        end
        st.if_th = wrap_pi(st.if_th + st.if_w*Ts);
        id_ref = single(p.Istart);
        iq_ref = single(0);
    end
    st.iq_ref = iq_ref;
    % hand off to closed loop once ramp completes
    handoff_done = (dir > single(0)) * (st.if_w >= w_hand) + (dir < single(0)) * (st.if_w <= w_hand);
    if (st.tmr >= single(p.Talign)) && handoff_done
        st.mode    = uint8(1);
        st.pll_th  = st.if_th;                  % seed estimator
        st.pll_int = st.if_w;
        st.w_est   = st.if_w;
        st.spd_int = iq;                        % bumpless: seed from measured iq
    end
else
    % ===== CLOSED-LOOP SPEED PI (mechanical units, decimated) =====
    if mod(st.step, uint32(p.Nspeed)) == uint32(0)
        wm_ref = TWO_PI*single(speed_ref_rpm)/single(60);   % mech rad/s
        wm_est = st.w_est/P;                                % mech rad/s
        e_w    = wm_ref - wm_est;
        st.spd_int = st.spd_int + single(p.Ki_w)*single(p.Tss)*e_w;
        st.spd_int = clamp(st.spd_int, -single(p.Imax), single(p.Imax));
        iq_ref = single(p.Kp_w)*e_w + st.spd_int;
        iq_ref = clamp(iq_ref, -single(p.Imax), single(p.Imax));
        st.iq_ref = iq_ref;
    else
        iq_ref = st.iq_ref;
    end
    id_ref = single(0);                          % SPMSM: no field weakening
end

% ---- 5. current PI controllers with feed-forward decoupling ----------
ed = id_ref - id;
st.id_int = st.id_int + single(p.Ki_i)*Ts*ed;
st.id_int = clamp(st.id_int, -Vmax, Vmax);
vd = single(p.Kp_i)*ed + st.id_int - we_est*Lq*iq;        % decouple

eq = iq_ref - iq;
st.iq_int = st.iq_int + single(p.Ki_i)*Ts*eq;
st.iq_int = clamp(st.iq_int, -Vmax, Vmax);
vq = single(p.Kp_i)*eq + st.iq_int + we_est*(Ld*id + single(p.Lambda)); % decouple

% ---- voltage magnitude limiting (circle limit) -----------------------
vmag = sqrt(vd*vd + vq*vq);
if vmag > Vmax
    sc = Vmax/vmag;
    vd = vd*sc;
    vq = vq*sc;
end

% ---- 6. inverse Park -------------------------------------------------
v_al = vd*ct - vq*sn;
v_be = vd*sn + vq*ct;

% ---- 7. flux observer + PLL (sensorless rotor tracking) --------------
% High-pass-compensated back-EMF integration (bounds DC drift).
emf_a = v_al - Rs*i_al;
emf_b = v_be - Rs*i_be;
st.psi_a = st.psi_a + Ts*(emf_a - single(p.Whp)*st.psi_a);
st.psi_b = st.psi_b + Ts*(emf_b - single(p.Whp)*st.psi_b);
% Active flux = stator flux minus leakage -> aligned to rotor d-axis.
fa = st.psi_a - Lq*i_al;
fb = st.psi_b - Lq*i_be;
fmag = sqrt(fa*fa + fb*fb) + single(1e-6);
% PLL phase detector: e ~ sin(theta_flux - pll_th)
e_pll = (-fa*sin(st.pll_th) + fb*cos(st.pll_th))/fmag;
st.pll_int = st.pll_int + single(p.Ki_pll)*Ts*e_pll;
w_pll      = single(p.Kp_pll)*e_pll + st.pll_int;
st.pll_th  = wrap_pi(st.pll_th + w_pll*Ts);
% low-pass the speed estimate
a_lp = single(p.Wspd_lp)*Ts;
st.w_est = st.w_est + a_lp*(w_pll - st.w_est);

% ---- 8. SVPWM (min/max common-mode injection) ------------------------
% inverse Clarke -> phase voltages
va = v_al;
vb = -single(0.5)*v_al + (SQ3/single(2))*v_be;
vc = -single(0.5)*v_al - (SQ3/single(2))*v_be;
vmaxp = max(max(va, vb), vc);
vminp = min(min(va, vb), vc);
vcom  = (vmaxp + vminp)/single(2);
inv_vdc = single(1)/single(vdc);
da = (va - vcom)*inv_vdc + single(0.5);
db = (vb - vcom)*inv_vdc + single(0.5);
dc = (vc - vcom)*inv_vdc + single(0.5);

if enable < single(0.5)
    da = single(0.5); db = single(0.5); dc = single(0.5);
end
duty = [clamp(da,single(0),single(1)), ...
        clamp(db,single(0),single(1)), ...
        clamp(dc,single(0),single(1))];

% ---- bookkeeping / debug ---------------------------------------------
st.step = st.step + uint32(1);
spd_rpm_est = st.w_est*single(60)/(TWO_PI*P);
dbg = [theta, spd_rpm_est, id, iq, vd, vq];
end

% ======================================================================
function y = clamp(x, lo, hi) %#codegen
y = x;
if y < lo, y = lo; end
if y > hi, y = hi; end
end

function y = wrap_pi(x) %#codegen
TWO_PI = single(2*pi);
y = x - TWO_PI*floor((x + single(pi))/TWO_PI);
end
