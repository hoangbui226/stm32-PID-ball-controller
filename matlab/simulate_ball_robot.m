%% SIMULATE_BALL_ROBOT
% Full closed-loop sim of the STM32 ball balancer.
% Pipeline (same order as main.c):
%   camera px -> EMA + deadband -> err_x/err_y
%   -> pid.c (P/I/D, LPF, theta/phi)
%   -> ik.c (deg1/deg2/deg3)
%   -> ik_to_servo (servo PWM commands)
%   -> simple gravity plant on the tilted plate
%
% After the run, every logged signal is in workspace struct `log`
% and also printed as a short summary. Open the figures to inspect.

clear
close all
clc

this_dir = fileparts(mfilename("fullpath"));
addpath(this_dir);

p = robot_params();

%% Time grids
dt_ctrl = 1 / p.hz;
dt = dt_ctrl / 10;                 % plant step (finer than control)
N = round(p.t_end / dt);
ctrl_every = round(dt_ctrl / dt);

%% State
ball = p.ball0;                   % cm on plate
vel = p.vel0;
ball_px = plate_to_pixels(ball, p);
filt_px = ball_px;                % EMA like main.c ball_x/ball_y

pid = struct("sum_err", [0 0], "prev_err", [0 0], "prev_out", [0 0]);

theta = 0; phi = 0;
theta_plant = 0; phi_plant = 0;
deg = [0 0 0];
servo = [62 62 62];
ik_ok = true;
joints = [];
pid_d = empty_pid_d();
ik_d = struct();

%% Preallocate log (one row per plant step; control fields hold last sample)
log = init_log(N);

%% Main loop
for i = 1:N
    t = (i - 1) * dt;

    if mod(i - 1, ctrl_every) == 0
        % --- camera sample ---
        ball_px = plate_to_pixels(ball, p);
        filt_px = filt_px + (ball_px - filt_px) * p.ball_ema;

        err_x = filt_px(1) - p.cam_cx;
        err_y = filt_px(2) - p.cam_cy;
        if abs(err_x) <= p.deadband, err_x = 0; end
        if abs(err_y) <= p.deadband, err_y = 0; end

        % --- PID (firmware) ---
        [theta, phi, pid, pid_d] = pid_step(pid, err_x, err_y, dt_ctrl, p);

        % --- plant tilt (optional 180 deg flip for stable gravity loop) ---
        theta_plant = theta;
        phi_plant = phi;
        if p.balance_flip
            phi_plant = mod(phi + 180, 360);
        end

        % --- IK + servo map ---
        [deg_new, ik_ok, joints_new, ik_d] = ik_solve(theta_plant, phi_plant, p);
        if ik_ok
            deg = deg_new;
            joints = joints_new;
            servo = ik_to_servo(deg, p.servo_offset, p);
        end
    end

    % --- plant: ball on tilted plane ---
    [ball, vel, plant] = step_plant(ball, vel, theta_plant, phi_plant, dt, p);

    % --- log everything ---
    log.t(i) = t;
    log.ball_cm(i, :) = ball;
    log.vel_cm_s(i, :) = vel;
    log.acc_cm_s2(i, :) = plant.acc;
    log.ball_px(i, :) = plate_to_pixels(ball, p);
    log.filt_px(i, :) = filt_px;
    log.err_px(i, :) = pid_d.err;
    log.sum_err(i, :) = pid_d.sum_err;
    log.d_err(i, :) = pid_d.d_err;
    log.p_term(i, :) = pid_d.p_term;
    log.i_term(i, :) = pid_d.i_term;
    log.d_term(i, :) = pid_d.d_term;
    log.raw(i, :) = pid_d.raw;
    log.filt(i, :) = pid_d.filt;
    log.radius(i) = pid_d.radius;
    log.theta(i) = theta;
    log.phi(i) = phi;
    log.theta_plant(i) = theta_plant;
    log.phi_plant(i) = phi_plant;
    log.deg(i, :) = deg;
    log.servo(i, :) = servo;
    log.ik_ok(i) = ik_ok;
    log.n_hat(i, :) = plant.n;
end

%% Put a tidy snapshot on the base workspace
assignin("base", "p", p);
assignin("base", "log", log);
assignin("base", "pid_state", pid);
assignin("base", "last_pid", pid_d);
assignin("base", "last_ik", ik_d);
assignin("base", "last_joints", joints);

print_summary(p, log);
plot_all(p, log);

fprintf("Playing 3D replay (close that window to continue)...\n");
fprintf("For click-to-place live control, run:  live_ball_robot\n");
animate_ball_robot(log, p, 1.5);

%% ========== helpers ==========

function log = init_log(N)
log.t = zeros(N, 1);
log.ball_cm = zeros(N, 2);
log.vel_cm_s = zeros(N, 2);
log.acc_cm_s2 = zeros(N, 2);
log.ball_px = zeros(N, 2);
log.filt_px = zeros(N, 2);
log.err_px = zeros(N, 2);
log.sum_err = zeros(N, 2);
log.d_err = zeros(N, 2);
log.p_term = zeros(N, 2);
log.i_term = zeros(N, 2);
log.d_term = zeros(N, 2);
log.raw = zeros(N, 2);
log.filt = zeros(N, 2);
log.radius = zeros(N, 1);
log.theta = zeros(N, 1);
log.phi = zeros(N, 1);
log.theta_plant = zeros(N, 1);
log.phi_plant = zeros(N, 1);
log.deg = zeros(N, 3);
log.servo = zeros(N, 3);
log.ik_ok = false(N, 1);
log.n_hat = zeros(N, 3);
end

function d = empty_pid_d()
d = struct( ...
    "err", [0 0], "sum_err", [0 0], "d_err", [0 0], ...
    "p_term", [0 0], "i_term", [0 0], "d_term", [0 0], ...
    "raw", [0 0], "filt", [0 0], "radius", 0, ...
    "theta", 0, "phi", 0, "dt", 0);
end

function px = plate_to_pixels(ball_cm, p)
% Plate cm -> fake camera pixels (origin at plate center = cam center).
px = [p.cam_cx + ball_cm(1) * p.px_per_cm, ...
      p.cam_cy + ball_cm(2) * p.px_per_cm];
end

function [ball, vel, plant] = step_plant(ball, vel, theta, phi, dt, p)
n = [sind(theta) * cosd(phi), sind(theta) * sind(phi), cosd(theta)];
n = n / max(norm(n), 1e-9);
g_world = [0, 0, -p.g];
g_plane = g_world - dot(g_world, n) * n;
acc = g_plane(1:2) - p.mu * vel;
vel = vel + acc * dt;
ball = ball + vel * dt;
r = hypot(ball(1), ball(2));
if r > p.plate_radius
    ball = ball * (p.plate_radius / r);
    vel = [0 0];
end
plant = struct("n", n, "acc", acc, "g_plane", g_plane);
end

function print_summary(p, log)
fprintf("\n=== Ball balancer sim ===\n");
fprintf("Control: %g Hz for %.1f s | balance_flip=%d\n", p.hz, p.t_end, p.balance_flip);
fprintf("PID gains: kp=%.4g  ki=%.4g  kd=%.4g  alpha=%.2f  beta=%.2f\n", ...
    p.kp, p.ki, p.kd, p.alpha, p.beta);
fprintf("Start ball (cm):  %6.2f %6.2f\n", p.ball0);
fprintf("Final ball (cm):  %6.2f %6.2f\n", log.ball_cm(end, :));
fprintf("Final pixels:     %6.1f %6.1f  (target %d %d)\n", ...
    log.filt_px(end, :), p.cam_cx, p.cam_cy);
fprintf("Final theta/phi:  %6.2f / %.1f deg (firmware)\n", log.theta(end), log.phi(end));
fprintf("Final IK horns:   %6.1f %6.1f %6.1f deg\n", log.deg(end, :));
fprintf("Final servos:     %6d %6d %6d  (D13 / D12 / D11)\n", log.servo(end, :));
fprintf("IK fail samples:  %d / %d\n", sum(~log.ik_ok), numel(log.ik_ok));
fprintf("\nWorkspace: log, p, pid_state, last_pid, last_ik, last_joints\n");
fprintf("Try:  plot(log.t, log.err_px)   or   openvar('log')\n\n");
end

function plot_all(p, log)
t = log.t;

% Figure 1 — plant + camera
figure("Name", "1. Ball & camera", "Color", "w")
subplot(2, 2, 1)
plot(log.ball_cm(:,1), log.ball_cm(:,2), "b-"); hold on
plot(0, 0, "r+", "MarkerSize", 12, "LineWidth", 1.5)
th = linspace(0, 2*pi, 100);
plot(p.plate_radius*cos(th), p.plate_radius*sin(th), "k-")
axis equal; grid on
xlabel("x (cm)"); ylabel("y (cm)"); title("Ball path on plate")

subplot(2, 2, 2)
plot(t, log.ball_cm); grid on
xlabel("t (s)"); ylabel("cm"); legend("x","y"); title("Ball position")

subplot(2, 2, 3)
plot(t, log.filt_px); hold on
yline(p.cam_cx, ":"); yline(p.cam_cy, ":")
grid on
xlabel("t (s)"); ylabel("px"); legend("filt_x","filt_y","cx","cy")
title(sprintf("Camera EMA (target %d,%d)", p.cam_cx, p.cam_cy))

subplot(2, 2, 4)
plot(t, log.err_px); grid on
xlabel("t (s)"); ylabel("px"); legend("err_x","err_y")
title(sprintf("Error after deadband (±%d)", p.deadband))

% Figure 2 — every PID internal
figure("Name", "2. PID internals (pid.c)", "Color", "w")
subplot(3, 2, 1)
plot(t, log.p_term); grid on; legend("Px","Py"); title("P term = kp*err")
subplot(3, 2, 2)
plot(t, log.i_term); grid on; legend("Ix","Iy"); title("I term = ki*sum")
subplot(3, 2, 3)
plot(t, log.d_term); grid on; legend("Dx","Dy"); title("D term = kd*d_err")
subplot(3, 2, 4)
plot(t, log.raw, "--"); hold on; plot(t, log.filt); grid on
legend("raw_x","raw_y","filt_x","filt_y"); title("Raw PID vs LPF (alpha)")
subplot(3, 2, 5)
plot(t, log.radius); grid on; title("radius = hypot(filt)")
xlabel("t (s)")
subplot(3, 2, 6)
yyaxis left; plot(t, log.theta); ylabel("theta (deg)")
yyaxis right; plot(t, log.phi); ylabel("phi (deg)")
grid on; title("Firmware theta / phi"); xlabel("t (s)")

% Figure 3 — IK + servos
figure("Name", "3. IK & servos", "Color", "w")
subplot(2, 2, 1)
plot(t, log.theta_plant, t, log.phi_plant); grid on
legend("theta_{plant}","phi_{plant}")
title(sprintf("Plant command (flip=%d)", p.balance_flip))
subplot(2, 2, 2)
plot(t, log.deg); grid on
legend("deg1 ArmA/D13","deg2 +X/D12","deg3 ArmC/D11")
title("ik.c horn angles")
subplot(2, 2, 3)
plot(t, log.servo); grid on
legend("servo A D13","servo B D12","servo C D11")
title("ik_to_servo = 90 - deg (clamped)")
xlabel("t (s)"); ylabel("deg")
subplot(2, 2, 4)
plot(t, log.ik_ok); ylim([-0.1 1.1]); grid on
title("ik_ok (1=solution)"); xlabel("t (s)")

% Figure 4 — dynamics
figure("Name", "4. Plant dynamics", "Color", "w")
subplot(2, 1, 1)
plot(t, log.vel_cm_s); grid on; legend("vx","vy"); title("Ball velocity (cm/s)")
subplot(2, 1, 2)
plot(t, log.acc_cm_s2); grid on; legend("ax","ay"); title("Ball accel on plane (cm/s^2)")
xlabel("t (s)")

end
