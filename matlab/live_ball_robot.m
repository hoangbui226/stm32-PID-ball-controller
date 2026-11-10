%% LIVE_BALL_ROBOT
% Interactive realtime view of the ball balancer.
%
%   Click INSIDE the gray plate circle  -> place the ball
%   Or use the buttons under the plots
%   R / Reset button -> center
%   Q / Quit button  -> quit
%
% Must run in the MATLAB desktop (Command Window), not PowerShell:
%   cd ...\matlab
%   live_ball_robot

clear
close all
clc

this_dir = fileparts(mfilename("fullpath"));
addpath(this_dir);
p = robot_params();

dt_ctrl = 1 / p.hz;
dt = 1 / 30;                          % UI-friendly frame rate
ctrl_every = max(1, round(dt_ctrl / dt));

ball = [0, 0];
vel = [0, 0];
filt_px = [p.cam_cx, p.cam_cy];
pid = struct("sum_err", [0 0], "prev_err", [0 0], "prev_out", [0 0]);
theta = 0; phi = 0;
theta_plant = 0; phi_plant = 0;
[deg, ok0, joints] = ik_solve(0, 0, p);
if ~ok0
    error("IK failed at theta=0 — check robot_params geometry.");
end
servo = ik_to_servo(deg, p.servo_offset, p);

%% Figure layout (manual axes so buttons work reliably)
fig = figure( ...
    "Name", "Live ball balancer", ...
    "Color", "w", ...
    "NumberTitle", "off", ...
    "MenuBar", "none", ...
    "ToolBar", "figure", ...
    "KeyPressFcn", @on_key, ...
    "WindowButtonDownFcn", @on_fig_click);
fig.UserData.quit = false;
fig.UserData.place = [];
fig.UserData.reset = false;
fig.UserData.plate_radius = p.plate_radius;

ax2 = axes("Parent", fig, "Units", "normalized", "Position", [0.06 0.20 0.40 0.72]);
hold(ax2, "on"); grid(ax2, "on"); axis(ax2, "equal")
thc = linspace(0, 2*pi, 120);
% Clickable plate disk (this is what receives the mouse hit)
disk = patch(ax2, p.plate_radius*cos(thc), p.plate_radius*sin(thc), [0.85 0.88 0.92], ...
    "EdgeColor", "k", "LineWidth", 1.2, ...
    "FaceAlpha", 0.9, ...
    "PickableParts", "all", ...
    "HitTest", "on", ...
    "ButtonDownFcn", @on_disk_click);
plot(ax2, 0, 0, "k+", "MarkerSize", 10, "HitTest", "off", "PickableParts", "none")
ball2 = plot(ax2, 0, 0, "ro", "MarkerFaceColor", "r", "MarkerSize", 14, ...
    "HitTest", "off", "PickableParts", "none");
title(ax2, "Click gray plate to place ball")
xlabel(ax2, "x (cm)"); ylabel(ax2, "y (cm)")
xlim(ax2, [-1 1]*p.plate_radius*1.2)
ylim(ax2, [-1 1]*p.plate_radius*1.2)
ax2.PickableParts = "all";
fig.UserData.ax2 = ax2;
fig.UserData.disk = disk;

ax3 = axes("Parent", fig, "Units", "normalized", "Position", [0.52 0.20 0.45 0.72]);
h3 = robot_draw(ax3, joints, ball, p, theta_plant, phi_plant);
title(ax3, "Realtime arms / plate")

% Buttons (always work even if axes callbacks fail)
uicontrol(fig, "Style", "pushbutton", "String", "Place [3, -2]", ...
    "Units", "normalized", "Position", [0.06 0.06 0.14 0.08], ...
    "FontSize", 11, "Callback", @(~,~) set_place(fig, [3, -2]));
uicontrol(fig, "Style", "pushbutton", "String", "Place [-3, 2]", ...
    "Units", "normalized", "Position", [0.22 0.06 0.14 0.08], ...
    "FontSize", 11, "Callback", @(~,~) set_place(fig, [-3, 2]));
uicontrol(fig, "Style", "pushbutton", "String", "Reset (R)", ...
    "Units", "normalized", "Position", [0.38 0.06 0.12 0.08], ...
    "FontSize", 11, "Callback", @(~,~) set_reset(fig));
uicontrol(fig, "Style", "pushbutton", "String", "Quit (Q)", ...
    "Units", "normalized", "Position", [0.52 0.06 0.12 0.08], ...
    "FontSize", 11, "Callback", @(~,~) set_quit(fig));

info = uicontrol(fig, "Style", "text", "Units", "normalized", ...
    "Position", [0.66 0.04 0.32 0.10], "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", "FontName", "Consolas", "FontSize", 9, ...
    "String", "Ready");

fprintf("Live view running.\n");
fprintf("  Click the gray circle, OR press Place / Reset / Quit buttons.\n");
fprintf("  Keep this figure focused for keyboard Q/R.\n");

step = 0;
while ishandle(fig) && ~fig.UserData.quit
    if ~isempty(fig.UserData.place)
        xy = fig.UserData.place;
        r = hypot(xy(1), xy(2));
        if r > p.plate_radius
            xy = xy * (p.plate_radius / r);
        end
        ball = xy;
        vel = [0 0];
        fig.UserData.place = [];
        fprintf("Placed ball at [%.2f, %.2f] cm\n", ball);
    end
    if fig.UserData.reset
        ball = [0 0];
        vel = [0 0];
        pid.sum_err = [0 0];
        pid.prev_err = [0 0];
        pid.prev_out = [0 0];
        filt_px = [p.cam_cx, p.cam_cy];
        fig.UserData.reset = false;
        fprintf("Reset to center\n");
    end

    if mod(step, ctrl_every) == 0
        px = [p.cam_cx + ball(1)*p.px_per_cm, p.cam_cy + ball(2)*p.px_per_cm];
        filt_px = filt_px + (px - filt_px) * p.ball_ema;
        err_x = filt_px(1) - p.cam_cx;
        err_y = filt_px(2) - p.cam_cy;
        if abs(err_x) <= p.deadband, err_x = 0; end
        if abs(err_y) <= p.deadband, err_y = 0; end

        [theta, phi, pid] = pid_step(pid, err_x, err_y, dt_ctrl, p);
        theta_plant = theta;
        phi_plant = phi;
        if p.balance_flip
            phi_plant = mod(phi + 180, 360);
        end
        [deg_new, ok, joints_new] = ik_solve(theta_plant, phi_plant, p);
        if ok
            deg = deg_new;
            joints = joints_new;
            servo = ik_to_servo(deg, p.servo_offset, p);
        end
    end

    n = [sind(theta_plant)*cosd(phi_plant), sind(theta_plant)*sind(phi_plant), cosd(theta_plant)];
    n = n / max(norm(n), 1e-9);
    g_plane = [0 0 -p.g] - dot([0 0 -p.g], n) * n;
    acc = g_plane(1:2) - p.mu * vel;
    vel = vel + acc * dt;
    ball = ball + vel * dt;
    r = hypot(ball(1), ball(2));
    if r > p.plate_radius
        ball = ball * (p.plate_radius / r);
        vel = [0 0];
    end

    set(ball2, "XData", ball(1), "YData", ball(2))
    robot_draw_update(h3, joints, ball, p, theta_plant, phi_plant)
    set(info, "String", sprintf([ ...
        "ball=[%.2f %.2f] cm\n" ...
        "theta=%.2f  phi=%.1f\n" ...
        "servo=[%d %d %d]"], ball, theta, phi, servo));

    drawnow                          % full drawnow so UI callbacks fire
    pause(dt)
    step = step + 1;
end

if ishandle(fig)
    close(fig)
end

%% ---- callbacks ----

function on_key(~, evt)
fig = gcbf;
if isempty(fig), return; end
switch upper(evt.Key)
    case {"Q", "ESCAPE"}
        fig.UserData.quit = true;
    case "R"
        fig.UserData.reset = true;
end
end

function on_disk_click(src, ~)
fig = ancestor(src, "figure");
ax = ancestor(src, "axes");
cp = ax.CurrentPoint;
xy = cp(1, 1:2);
R = fig.UserData.plate_radius;
if hypot(xy(1), xy(2)) <= R * 1.01
    fig.UserData.place = xy;
end
end

function on_fig_click(fig, ~)
% Fallback: if user clicked in the left axes area, place there.
ax2 = fig.UserData.ax2;
if ~ishandle(ax2), return; end
hit = get(fig, "CurrentObject");
if isempty(hit), return; end
if ~(hit == ax2 || isequal(ancestor(hit, "axes"), ax2) || hit == fig.UserData.disk)
    return
end
cp = ax2.CurrentPoint;
xy = cp(1, 1:2);
R = fig.UserData.plate_radius;
if hypot(xy(1), xy(2)) <= R * 1.01
    fig.UserData.place = xy;
end
end

function set_place(fig, xy)
if ishandle(fig)
    fig.UserData.place = xy;
end
end

function set_reset(fig)
if ishandle(fig)
    fig.UserData.reset = true;
end
end

function set_quit(fig)
if ishandle(fig)
    fig.UserData.quit = true;
end
end
