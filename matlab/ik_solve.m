function [deg, ok, joints, d] = ik_solve(theta_deg, phi_deg, p)
% IK_SOLVE  Inverse kinematics matching Core/Src/ik.c
%
% deg     = [deg1 deg2 deg3] horn commands (same meaning as ik.c outs)
%           deg1 = Arm A (120 deg plate), deg2 = +X arm, deg3 = 240 deg arm
% ok      = false if geometry has no real solution
% joints  = A/B/C points (cm) for drawing
% d       = every intermediate scalar used inside the solve

deg = [NaN NaN NaN];
ok = false;
joints = struct("A", [], "B", [], "C", []);
d = struct();

theta_clamped = min(max(theta_deg, 0), p.max_theta);
th = deg2rad(theta_clamped);
ph = deg2rad(phi_deg);

a = sin(th) * cos(ph);
b = sin(th) * sin(ph);
c = cos(th);

s3 = sqrt(3);
q1 = a - s3 * b;
q3 = a + s3 * b;
d1 = sqrt(4 * c^2 + q1^2);
d2 = sqrt(c^2 + a^2);
d3 = sqrt(4 * c^2 + q3^2);

d.theta_in = theta_deg;
d.phi_in = phi_deg;
d.theta_clamped = theta_clamped;
d.a = a; d.b = b; d.c = c;
d.q1 = q1; d.q3 = q3;
d.d1 = d1; d.d2 = d2; d.d3 = d3;

if d1 < 1e-4 || d2 < 1e-4 || d3 < 1e-4
    return
end

A1 = [-(p.lp * c) / d1, (s3 * p.lp * c) / d1, p.h + (p.lp * q1) / d1];
A2 = [(p.lp * c) / d2, 0, p.h - (p.lp * a) / d2];
A3 = [-(p.lp * c) / d3, -(s3 * p.lp * c) / d3, p.h + (p.lp * q3) / d3];
d.A1 = A1; d.A2 = A2; d.A3 = A3;

if abs(A1(3)) < 1e-4 || abs(A2(3)) < 1e-4 || abs(A3(3)) < 1e-4
    return
end

p1 = (-A1(1) + s3 * A1(2) - 2 * p.lb) / A1(3);
v1 = (sum(A1.^2) + p.l2^2 - p.l1^2 - p.lb^2) / (2 * A1(3));
r1 = p1^2 + 4;
s1 = 2 * p1 * v1 + 4 * p.lb;
t1 = v1^2 + p.lb^2 - p.l2^2;

p2 = (p.lb - A2(1)) / A2(3);
v2 = (A2(1)^2 + A2(3)^2 - p.lb^2 + p.l2^2 - p.l1^2) / (2 * A2(3));
r2 = p2^2 + 1;
s2 = 2 * (p2 * v2 - p.lb);
t2 = v2^2 - p.l2^2 + p.lb^2;

p3v = (-A3(1) - s3 * A3(2) - 2 * p.lb) / A3(3);
v3 = (sum(A3.^2) + p.l2^2 - p.l1^2 - p.lb^2) / (2 * A3(3));
r3 = p3v^2 + 4;
s3q = 2 * p3v * v3 + 4 * p.lb;
t3 = v3^2 + p.lb^2 - p.l2^2;

disc1 = s1^2 - 4 * r1 * t1;
disc2 = s2^2 - 4 * r2 * t2;
disc3 = s3q^2 - 4 * r3 * t3;

d.disc = [disc1 disc2 disc3];
if disc1 < 0 || disc2 < 0 || disc3 < 0
    return
end

c11 = (-s1 - sqrt(disc1)) / (2 * r1);
c21 = (-s2 + sqrt(disc2)) / (2 * r2);
c31 = (-s3q - sqrt(disc3)) / (2 * r3);
u1 = p.l2^2 - 4 * c11^2 - 4 * p.lb * c11 - p.lb^2;
u2 = p.l2^2 - (c21 - p.lb)^2;
u3 = p.l2^2 - 4 * c31^2 - 4 * p.lb * c31 - p.lb^2;
d.under = [u1 u2 u3];
if u1 < 0 || u2 < 0 || u3 < 0
    return
end

C1 = [c11, -s3 * c11, sqrt(u1)];
C2 = [c21, 0, sqrt(u2)];
C3 = [c31, s3 * c31, sqrt(u3)];

% Same formulas as ik.c outs
deg1 = atan2(hypot(C1(1), C1(2)) - p.lb, C1(3)) * (180 / pi);
deg2 = (pi/2 - atan2(C2(3), C2(1) - p.lb)) * (180 / pi);
deg3 = atan2(hypot(C3(1), C3(2)) - p.lb, C3(3)) * (180 / pi);

deg = [deg1, deg2, deg3];
ok = true;
d.deg = deg;

joints.A = [A1; A2; A3];
joints.B = [-0.5*p.lb, s3*0.5*p.lb, 0;
             p.lb, 0, 0;
            -0.5*p.lb, -s3*0.5*p.lb, 0];
joints.C = [C1; C2; C3];
end
