function [ia, ib, ic, theta_e, wm_rpm, st] = pmsm_plant_step(duty, vdc, Tload, st, p) %#codegen
%PMSM_PLANT_STEP Surface-mount PMSM + averaged-inverter plant (sim only).
%   Outputs all three phase currents (three-shunt sensing model).
%
%   The current/angle OUTPUTS are taken from the state BEFORE the duty input
%   is applied this tick, so combined with a 1-sample control delay there is
%   no algebraic loop with the controller.

Rs = p.Rs; Ld = p.Ld; Lq = p.Lq; lam = p.Lambda;
P  = p.PolePairs; J = p.J; B = p.B;
Ts = p.Ts;
NSUB = 4;                       % integration sub-steps per control tick
dt = Ts/NSUB;

% ----- outputs from current state (pre-update) ------------------------
id = st.id; iq = st.iq; th_e = st.th_e;
i_al = id*cos(th_e) - iq*sin(th_e);
i_be = id*sin(th_e) + iq*cos(th_e);
ia = i_al;
ib = -0.5*i_al + (sqrt(3)/2)*i_be;
ic = -0.5*i_al - (sqrt(3)/2)*i_be;
theta_e = th_e;
wm_rpm  = st.wm*60/(2*pi);

% ----- averaged inverter: duty -> phase voltages (vs neutral) ---------
davg = (duty(1) + duty(2) + duty(3))/3;
va = (duty(1) - davg)*vdc;
vb = (duty(2) - davg)*vdc;
vc = (duty(3) - davg)*vdc;
% Clarke (amplitude invariant)
v_al = va;
v_be = (vb - vc)/sqrt(3);

% ----- integrate dq dynamics ------------------------------------------
idk = st.id; iqk = st.iq; wmk = st.wm; thk = st.th_e;
for k = 1:NSUB
    we = P*wmk;
    ct = cos(thk); sn = sin(thk);
    vd =  v_al*ct + v_be*sn;
    vq = -v_al*sn + v_be*ct;
    did = (vd - Rs*idk + we*Lq*iqk)/Ld;
    diq = (vq - Rs*iqk - we*(Ld*idk + lam))/Lq;
    Te  = 1.5*P*(lam*iqk + (Ld - Lq)*idk*iqk);
    dwm = (Te - B*wmk - Tload)/J;
    idk = idk + dt*did;
    iqk = iqk + dt*diq;
    wmk = wmk + dt*dwm;
    thk = thk + dt*we;
end
thk = mod(thk + pi, 2*pi) - pi;

st.id = idk; st.iq = iqk; st.wm = wmk; st.th_e = thk;
end
