function st = foc_state_init(p) %#codegen
%FOC_STATE_INIT Initialize the single-precision FOC controller state struct.
%   All fields are scalar 'single' so the struct maps cleanly to a C struct
%   for the STM32F429.
z = single(0);
st = struct( ...
    'mode',      uint8(0), ...   % 0 = I/F open-loop startup, 1 = closed loop
    'step',      uint32(0), ...  % step counter (for speed-loop decimation)
    'id_int',    z, ...          % d-current PI integrator      [V]
    'iq_int',    z, ...          % q-current PI integrator      [V]
    'spd_int',   z, ...          % speed PI integrator          [A]
    'psi_a',     z, ...          % stator flux alpha            [Wb]
    'psi_b',     z, ...          % stator flux beta             [Wb]
    'pll_th',    z, ...          % PLL angle (electrical)       [rad]
    'pll_int',   z, ...          % PLL integrator -> speed      [rad/s]
    'w_est',     z, ...          % filtered electrical speed    [rad/s]
    'if_th',     z, ...          % I/F forced angle             [rad]
    'if_w',      z, ...          % I/F forced electrical speed  [rad/s]
    'iq_ref',    z, ...          % last iq reference (bumpless) [A]
    'tmr',       z );            % mode-0 elapsed time          [s]
end
