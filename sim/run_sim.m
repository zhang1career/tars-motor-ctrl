function out = run_sim(stopTime)
%RUN_SIM Simulate pmsm_ctrl.slx and save result plots to results/.
if nargin < 1, stopTime = 1.2; end
here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'pmsm_ctrl';
load_system(fullfile(here,[mdl '.slx']));
set_param(mdl,'StopTime',num2str(stopTime));

out = sim(mdl);

% ----- extract signals -----
sa  = out.speed_act;   t = sa.Time;
se  = out.speed_est;
sr  = out.speed_ref;
idq = out.idq;         % [id iq]
the = out.theta_e;
thh = out.theta_est;

id = squeeze(idq.Data(:,1)); iq = squeeze(idq.Data(:,2));
% wrap angle error to [-pi,pi]
ae = thh.Data - the.Data;
ae = atan2(sin(ae), cos(ae));

% ----- print summary -----
fprintf('--- SIM SUMMARY (Ts=%g, stop=%g) ---\n', t(2)-t(1), stopTime);
fprintf('final actual speed = %.1f rpm (ref %.1f)\n', sa.Data(end), sr.Data(end));
fprintf('final est speed    = %.1f rpm\n', se.Data(end));
i2 = find(t>0.6,1);                      % steady state window start
fprintf('mean angle err (t>0.6s) = %.2f deg, std = %.2f deg\n', ...
    mean(ae(i2:end))*180/pi, std(ae(i2:end))*180/pi);
fprintf('steady iq = %.3f A, id = %.3f A\n', mean(iq(i2:end)), mean(id(i2:end)));

% ----- plots -----
f = figure('Position',[100 100 1100 800],'Visible','off');
subplot(3,1,1);
plot(t, sr.Data,'k--', t, sa.Data,'b', t, se.Data,'r','LineWidth',1.2); grid on;
legend('ref','actual','estimated','Location','southeast');
ylabel('speed [rpm]'); title('Sensorless Speed-FOC: speed tracking (I/F startup -> closed loop)');

subplot(3,1,2);
plot(t, id,'b', t, iq,'r','LineWidth',1.0); grid on;
legend('i_d','i_q','Location','northeast'); ylabel('current [A]');
title('dq currents');

subplot(3,1,3);
plot(t, ae*180/pi,'m','LineWidth',1.0); grid on;
ylabel('angle error [deg]'); xlabel('time [s]');
title('estimated vs actual rotor angle error'); ylim([-60 60]);

if ~exist(fullfile(here,'results'),'dir'), mkdir(fullfile(here,'results')); end
png = fullfile(here,'results','sim_results.png');
exportgraphics(f, png, 'Resolution', 130);
fprintf('Saved %s\n', png);
close(f);
end
