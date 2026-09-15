function st = pmsm_plant_init(p) %#codegen
%PMSM_PLANT_INIT Initialize PMSM dq-plant state (double precision, sim only).
st = struct( ...
    'id',     0, ...   % d-axis current  [A]
    'iq',     0, ...   % q-axis current  [A]
    'wm',     0, ...   % mechanical speed [rad/s]
    'th_e',   0 );     % electrical angle [rad]
end
