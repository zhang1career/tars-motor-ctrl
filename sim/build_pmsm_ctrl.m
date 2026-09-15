function build_pmsm_ctrl()
%BUILD_PMSM_CTRL Programmatically build pmsm_ctrl.slx
%   Sensorless speed-FOC controller + PMSM/averaged-inverter plant, wired
%   with a 1-sample control delay (ADC->PWM latency) to break the loop.

here = fileparts(mfilename('fullpath'));
addpath(here);
p = mc_params();
mdl = 'pmsm_ctrl';

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here,[mdl '.slx']),'file'), delete(fullfile(here,[mdl '.slx'])); end
new_system(mdl);
load_system(mdl);

% ---------- sources ----------
add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Value', num2str(p.Vdc), 'Position', [30 40 70 70]);
add_block('simulink/Sources/Constant', [mdl '/SpeedRef'], ...
    'Value', '1500', 'Position', [30 120 70 150]);
add_block('simulink/Sources/Constant', [mdl '/Enable'], ...
    'Value', '1', 'Position', [30 200 70 230]);
add_block('simulink/Sources/Step', [mdl '/Tload'], ...
    'Time', '1.0', 'Before', '0', 'After', '0.10', 'Position', [30 280 70 310]);

% ---------- controller (MATLAB Function) ----------
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/foc_ctrl'], 'Position', [200 60 320 200]);
set_fcn_block([mdl '/foc_ctrl'], ctrl_code());

% ---------- 1-sample control delay on duty ----------
add_block('simulink/Discrete/Unit Delay', [mdl '/Zdelay'], ...
    'X0', '[0.5 0.5 0.5]', 'SampleTime', '-1', 'Position', [400 70 440 100]);

% ---------- plant (MATLAB Function) ----------
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/plant'], 'Position', [520 90 640 220]);
set_fcn_block([mdl '/plant'], plant_code());

% ---------- logging ----------
logs = {'speed_act','speed_est','idq','theta_e','theta_est','speed_ref'};
ypos = 40;
for i = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' logs{i}], ...
        'VariableName', logs{i}, 'SaveFormat', 'Timeseries', ...
        'SampleTime', '-1', 'Position', [760 ypos 820 ypos+30]);
    ypos = ypos + 60;
end
% mux for id,iq
add_block('simulink/Signal Routing/Mux', [mdl '/muxIdq'], 'Inputs', '2', ...
    'Position', [690 250 695 290]);

% ---------- wiring ----------
% controller inputs: 1 ia, 2 ib, 3 ic, 4 vdc, 5 speed_ref_rpm, 6 enable
add_line(mdl, 'Vdc/1',      'foc_ctrl/4', 'autorouting','on');
add_line(mdl, 'SpeedRef/1', 'foc_ctrl/5', 'autorouting','on');
add_line(mdl, 'Enable/1',   'foc_ctrl/6', 'autorouting','on');
% controller outputs: 1 duty, 2 theta_est, 3 speed_est, 4 id, 5 iq
add_line(mdl, 'foc_ctrl/1', 'Zdelay/1', 'autorouting','on');
% plant inputs: 1 duty, 2 vdc, 3 Tload
add_line(mdl, 'Zdelay/1',   'plant/1', 'autorouting','on');
add_line(mdl, 'Vdc/1',      'plant/2', 'autorouting','on');
add_line(mdl, 'Tload/1',    'plant/3', 'autorouting','on');
% plant outputs: 1 ia, 2 ib, 3 ic, 4 theta_e, 5 speed_rpm
add_line(mdl, 'plant/1', 'foc_ctrl/1', 'autorouting','on');
add_line(mdl, 'plant/2', 'foc_ctrl/2', 'autorouting','on');
add_line(mdl, 'plant/3', 'foc_ctrl/3', 'autorouting','on');
% logging
add_line(mdl, 'plant/5',    'log_speed_act/1', 'autorouting','on');
add_line(mdl, 'foc_ctrl/3', 'log_speed_est/1', 'autorouting','on');
add_line(mdl, 'foc_ctrl/4', 'muxIdq/1', 'autorouting','on');
add_line(mdl, 'foc_ctrl/5', 'muxIdq/2', 'autorouting','on');
add_line(mdl, 'muxIdq/1',   'log_idq/1', 'autorouting','on');
add_line(mdl, 'plant/3',    'log_theta_e/1', 'autorouting','on');
add_line(mdl, 'foc_ctrl/2', 'log_theta_est/1', 'autorouting','on');
add_line(mdl, 'SpeedRef/1', 'log_speed_ref/1', 'autorouting','on');

% ---------- solver ----------
set_param(mdl, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
    'FixedStep', num2str(p.Ts), 'StopTime', '1.2');

% realize ports / catch build errors early
set_param(mdl, 'SimulationCommand', 'update');
save_system(mdl, fullfile(here,[mdl '.slx']));
close_system(mdl, 0);
fprintf('Built %s OK\n', fullfile(here,[mdl '.slx']));
end

% ======================================================================
function set_fcn_block(blkPath, code)
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','-and','Path',blkPath);
ch.Script = code;
end

function c = ctrl_code()
c = sprintf([ ...
'function [duty, theta_est, speed_est, id, iq] = foc_ctrl(ia, ib, ic, vdc, speed_ref_rpm, enable)\n' ...
'%%#codegen\n' ...
'persistent st p initialized\n' ...
'if isempty(initialized)\n' ...
'    p = mc_params();\n' ...
'    st = foc_state_init(p);\n' ...
'    initialized = true;\n' ...
'end\n' ...
'[duty_s, dbg, st] = foc_controller_step(single(ia), single(ib), single(ic), single(vdc), single(speed_ref_rpm), single(enable), st, p);\n' ...
'%% cast to double at the Simulink boundary (core stays single for STM32 C)\n' ...
'duty = double(duty_s);\n' ...
'theta_est = double(dbg(1));\n' ...
'speed_est = double(dbg(2));\n' ...
'id = double(dbg(3));\n' ...
'iq = double(dbg(4));\n' ...
'end\n']);
end

function c = plant_code()
c = sprintf([ ...
'function [ia, ib, ic, theta_e, speed_rpm] = plant(duty, vdc, Tload)\n' ...
'persistent st p initialized\n' ...
'if isempty(initialized)\n' ...
'    p = mc_params();\n' ...
'    st = pmsm_plant_init(p);\n' ...
'    initialized = true;\n' ...
'end\n' ...
'[ia, ib, ic, theta_e, speed_rpm, st] = pmsm_plant_step(duty, vdc, Tload, st, p);\n' ...
'end\n']);
end
