function animate_ball_robot(log, p, speed)
% ANIMATE_BALL_ROBOT  Replay a simulate_ball_robot log in 3D.
%
%   animate_ball_robot(log, p)         % realtime
%   animate_ball_robot(log, p, 2)      % 2x speed

if nargin < 3 || isempty(speed)
    speed = 1;
end

% Subsample so drawing stays smooth
dt = median(diff(log.t));
stride = max(1, round((1/30) / max(dt, 1e-4)));  % ~30 fps
idx = 1:stride:numel(log.t);

fig = figure("Name", "Animated replay", "Color", "w", "NumberTitle", "off");
ax = axes("Parent", fig);
[~, ~, joints0] = ik_solve(log.theta_plant(1), log.phi_plant(1), p);
h = robot_draw(ax, joints0, log.ball_cm(1,:), p, log.theta_plant(1), log.phi_plant(1));
title(ax, "Replay — close window to stop")
info = annotation(fig, "textbox", [0.01 0.01 0.98 0.05], ...
    "String", "", "EdgeColor", "none", "FontName", "Consolas", "FontSize", 9);

for k = idx
    if ~ishandle(fig)
        break
    end
    th = log.theta_plant(k);
    ph = log.phi_plant(k);
    [~, ok, joints] = ik_solve(th, ph, p);
    if ~ok
        continue
    end
    robot_draw_update(h, joints, log.ball_cm(k,:), p, th, ph)
    info.String = sprintf("t=%.2fs   ball=[%.2f %.2f]   theta=%.2f  phi=%.1f   servo=[%d %d %d]", ...
        log.t(k), log.ball_cm(k,:), log.theta(k), log.phi(k), log.servo(k,:));
    drawnow limitrate
    pause(dt * stride / speed)
end
end
