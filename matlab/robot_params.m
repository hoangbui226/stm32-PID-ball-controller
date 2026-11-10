function p = robot_params()
% ROBOT_PARAMS  Numbers copied from the STM32 firmware (and a simple plant).
% Edit gains here, then re-run simulate_ball_robot.

%% Geometry from Core/Src/ik.c
p.lp = 7.20;              % top_radius  (cm)
p.l1 = 7.30;              % upper_arm   (cm)
p.l2 = 4.60;              % lower_arm   (cm)
p.lb = 4.00;              % bottom_radius (cm)
p.h  = 9.50;              % height      (cm)
p.max_theta = 16.0;       % ik.c clamp on theta_deg
p.plate_radius = 8.6;     % acrylic plate radius (cm), for the plant only

%% Camera / error path from Core/Src/main.c
p.cam_cx = 160;
p.cam_cy = 130;
p.cam_w = 320;
p.cam_h = 240;
p.px_per_cm = 14;         % approximate QVGA scale over the plate
p.deadband = 4;           % pixels
p.ball_ema = 0.6;         % main.c: ball += (x-ball)*6/10

%% PID  — edit these while tuning (same roles as Core/Src/pid.c)
% Demo PD that settles in this plant. EMA + LPF add lag, so kd=0 diverges.
% To mirror today's firmware exactly, use: kp=0.020, ki=0, kd=0, alpha=0.8, beta=0.45
p.kp = 0.012;
p.ki = 0.00000;
p.kd = 0.008;
p.alpha = 0.8;            % output LPF: alpha*new + (1-alpha)*old
p.beta = 0.45;            % tanh scale
p.theta_scale = 17.0;     % pid.c: 17*tanh(beta*radius)
p.max_theta_pid = 17.0;   % soft cap after tanh (ik still clamps to max_theta)

%% Servo map from main.c ik_to_servo / deg_to_us
p.servo_offset = 0;       % ik_to_servo(..., offset)
p.servo_min = 19;
p.servo_max = 90;

%% Closed-loop plant helpers (not in firmware)
% Firmware phi points the plate normal's horizontal part along the error.
% On a gravity plant that makes the ball side LOW. Set balance_flip=true so
% the simulated ball is driven back to center (common for this mechanism).
% Set false to see raw firmware tilt direction on the plant.
p.balance_flip = true;

p.g = 981;                % cm/s^2
p.mu = 0.05;              % light rolling drag
p.hz = 50;                % control rate (Hz)
p.t_end = 8;              % simulation length (s)

%% Initial ball state on the plate (cm). Camera +x right, +y down.
p.ball0 = [3.0, -2.0];
p.vel0  = [0, 0];
end
