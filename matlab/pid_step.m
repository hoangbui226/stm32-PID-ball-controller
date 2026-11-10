function [theta, phi, s, d] = pid_step(s, err_x, err_y, dt, p)
% PID_STEP  One control step matching Core/Src/pid.c.
%
% Inputs
%   s      persistent PID state (sum / prev)
%   err_x  pixel error after deadband (ball_x - cam_cx)
%   err_y  pixel error after deadband (ball_y - cam_cy)
%   dt     sample time (s)
%   p      robot_params()
%
% Outputs
%   theta, phi   plate lean (deg), same as pid.c
%   s            updated state
%   d            struct of every intermediate used this step

if dt <= 0
    dt = 0.001;
end

err = [err_x, err_y];

% Integral
s.sum_err = s.sum_err + err * dt;

% Derivative
d_err = (err - s.prev_err) / dt;

% P + I + D
p_term = p.kp * err;
i_term = p.ki * s.sum_err;
d_term = p.kd * d_err;
raw = p_term + i_term + d_term;

% Output LPF
filt = p.alpha * raw + (1 - p.alpha) * s.prev_out;

% Spherical plate command
phi = atan2d(filt(2), filt(1));
if phi < 0
    phi = phi + 360;
end
radius = hypot(filt(1), filt(2));
theta = p.theta_scale * tanh(p.beta * radius);
theta = min(max(theta, 0), p.max_theta_pid);

s.prev_err = err;
s.prev_out = filt;

d = struct();
d.err = err;
d.sum_err = s.sum_err;
d.d_err = d_err;
d.p_term = p_term;
d.i_term = i_term;
d.d_term = d_term;
d.raw = raw;
d.filt = filt;
d.radius = radius;
d.theta = theta;
d.phi = phi;
d.dt = dt;
end
