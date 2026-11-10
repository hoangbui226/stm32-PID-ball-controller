%% GEOGEBRA_ARM
% Interactive 2D arm sketch like your GeoGebra file — one balancer leg
% in the vertical plane (the +X arm from ik.c).
%
%   Drag B around A   -> changes plate lean (theta)
%   Or use the theta slider
%   Q to quit
%
% Geometry (cm) matches robot_params / ik.c:
%   E origin, F servo pivot, G elbow, D top joint, A plate center
%
% Run in MATLAB Command Window:
%   cd ...\matlab
%   geogebra_arm

clear
close all
clc

this_dir = fileparts(mfilename("fullpath"));
addpath(this_dir);
p = robot_params();

% --- lengths (same names as GeoGebra / ik) ---
lb = p.lb;    % E--F  bottom radius
l2 = p.l2;    % F--G  lower horn
l1 = p.l1;    % G--D  upper arm
lp = p.lp;    % A--D  top radius
h0 = p.h;     % nominal plate height

theta = 8;    % initial lean (deg), positive = D side down in this 2D view

fig = figure("Name", "GeoGebra-style arm (ik.c +X leg)", ...
    "Color", "w", "NumberTitle", "off", ...
    "MenuBar", "none", "ToolBar", "figure", ...
    "KeyPressFcn", @(~,e) on_key(e), ...
    "WindowButtonUpFcn", @(~,~) on_up(), ...
    "CloseRequestFcn", @(src,~) on_close(src));

fig.UserData.quit = false;
fig.UserData.dragging = false;
fig.UserData.theta = theta;
fig.UserData.p = p;

% Layout: algebra panel | drawing
ax = axes("Parent", fig, "Units", "normalized", "Position", [0.34 0.14 0.62 0.78]);
hold(ax, "on"); grid(ax, "on"); axis(ax, "equal")
xlabel(ax, "x (cm)"); ylabel(ax, "z (cm)")
title(ax, "Drag B (blue) around A — or use slider")
xlim(ax, [-14 6]); ylim(ax, [-1 14])

panel = uipanel(fig, "Title", "Algebra", "Units", "normalized", ...
    "Position", [0.02 0.14 0.28 0.78], "BackgroundColor", "w", ...
    "FontSize", 11, "FontWeight", "bold");
alg = uicontrol(panel, "Style", "text", "Units", "normalized", ...
    "Position", [0.04 0.02 0.92 0.96], "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", "FontName", "Consolas", "FontSize", 10, ...
    "String", "");

% Slider for theta
uicontrol(fig, "Style", "text", "Units", "normalized", ...
    "Position", [0.34 0.04 0.08 0.05], "String", "theta", ...
    "BackgroundColor", "w", "FontSize", 11);
sld = uicontrol(fig, "Style", "slider", "Units", "normalized", ...
    "Position", [0.42 0.045 0.40 0.04], ...
    "Min", 0, "Max", p.max_theta, "Value", theta, ...
    "Callback", @on_slider);
uicontrol(fig, "Style", "pushbutton", "String", "Quit", ...
    "Units", "normalized", "Position", [0.84 0.035 0.10 0.055], ...
    "FontSize", 11, "Callback", @(~,~) on_quit());

% Artists
thc = linspace(0, 2*pi, 180);
h_circF = plot(ax, nan, nan, "-", "Color", [0.85 0.75 0.2], "LineWidth", 0.8); % reach of lower
h_circD = plot(ax, nan, nan, "-", "Color", [0.9 0.4 0.6], "LineWidth", 0.8);  % reach of upper
h_circA = plot(ax, nan, nan, "-", "Color", [0.3 0.5 0.9], "LineWidth", 1.0);  % plate radius

h_EF = plot(ax, nan, nan, "-", "Color", [0.1 0.7 0.3], "LineWidth", 4); % green base
h_FG = plot(ax, nan, nan, "-", "Color", [0.95 0.75 0.1], "LineWidth", 4); % yellow lower
h_GD = plot(ax, nan, nan, "-", "Color", [0.95 0.35 0.55], "LineWidth", 4); % pink upper
h_AD = plot(ax, nan, nan, "-", "Color", [0.2 0.45 0.95], "LineWidth", 2.5); % blue plate spoke
h_AE = plot(ax, nan, nan, ":", "Color", [0.5 0.5 0.5], "LineWidth", 1);

h_E = plot(ax, 0, 0, "ko", "MarkerFaceColor", "k", "MarkerSize", 8);
h_F = plot(ax, nan, nan, "o", "Color", [0.1 0.7 0.3], "MarkerFaceColor", [0.1 0.7 0.3], "MarkerSize", 9);
h_G = plot(ax, nan, nan, "o", "Color", [0.95 0.75 0.1], "MarkerFaceColor", [0.95 0.75 0.1], "MarkerSize", 9);
h_D = plot(ax, nan, nan, "o", "Color", [0.95 0.35 0.55], "MarkerFaceColor", [0.95 0.35 0.55], "MarkerSize", 9);
h_A = plot(ax, nan, nan, "o", "Color", [0.2 0.2 0.2], "MarkerFaceColor", [0.85 0.85 0.85], "MarkerSize", 10);
% Draggable B (defines lean direction on circle around A)
h_B = plot(ax, nan, nan, "o", "Color", [0.15 0.35 0.95], "MarkerFaceColor", [0.35 0.55 1], ...
    "MarkerSize", 12, "PickableParts", "all", "HitTest", "on", ...
    "ButtonDownFcn", @(~,~) on_down());

h_txt = gobjects(5, 1);
for ii = 1:5
    h_txt(ii) = text(ax, 0, 0, "", "FontSize", 11, "FontWeight", "bold");
end

fig.UserData.h = struct("circF", h_circF, "circD", h_circD, "circA", h_circA, ...
    "EF", h_EF, "FG", h_FG, "GD", h_GD, "AD", h_AD, "AE", h_AE, ...
    "E", h_E, "F", h_F, "G", h_G, "D", h_D, "A", h_A, "B", h_B, ...
    "txt", h_txt, "alg", alg, "sld", sld, "ax", ax);
fig.UserData.thc = thc;
fig.UserData.lb = lb; fig.UserData.l1 = l1; fig.UserData.l2 = l2;
fig.UserData.lp = lp; fig.UserData.h0 = h0;

fig.WindowButtonMotionFcn = @(~,~) on_move();

redraw(fig);
fprintf("GeoGebra-style arm ready.\n");
fprintf("  Drag the blue point B around A, or move the theta slider.\n");

% Keep callbacks alive (figure-driven; no busy loop)
uiwait(fig);

%% ===== kinematics in the GeoGebra plane (x,z), mirrored like your sketch =====
% Servo pivot on -x (F = (-lb, 0)). Plate lean theta uses ik.c arm-2 formulas
% with phi = 180 so +x error raises the +x side; here we draw mirrored.

function [E, F, G, D, A, B, beta_deg, alpha_deg, ok] = arm_pose(theta_deg, ud)
lb = ud.lb; l1 = ud.l1; l2 = ud.l2; lp = ud.lp; h0 = ud.h0;
p = ud.p;

% Use real IK for the +X arm (arm index 2), then mirror x for GeoGebra view
[~, ok, joints] = ik_solve(theta_deg, 180, p);  % phi=180 -> lean in xz
E = [0, 0];
if ~ok
    F = [-lb, 0]; G = F; D = F; A = [0, h0]; B = A; beta_deg = NaN; alpha_deg = NaN;
    return
end

% Arm 2 points are in 3D with y=0: (x,z) from (x,0,z)
A3 = joints.A(2, :);   % top joint A2
B3 = joints.B(2, :);   % base pivot
C3 = joints.C(2, :);   % elbow
% Mirror to left like the GeoGebra figure
F = [-abs(B3(1)), B3(3)];
G = [-C3(1), C3(3)];
D = [-A3(1), A3(3)];
A = [0, h0];           % plate center (approx fixed height like your sketch)
% B = handle on circle around A, aimed toward D (plate spoke)
v = D - A;
if norm(v) < 1e-6
    ang = pi;
else
    ang = atan2(v(2), v(1));
end
B = A + lp * [cos(ang), sin(ang)];

% Angles
f = F; g = G; d = D;
vf = g - f;           % lower link
vg = d - g;           % upper link
beta_deg = atan2d(vf(2), vf(1));          % horn angle from +x (mirrored frame)
if beta_deg < 0, beta_deg = beta_deg + 360; end
% Interior angle at G between F-G-D
a1 = atan2d(vf(2), vf(1));
a2 = atan2d(vg(2), vg(1));
alpha_deg = abs(wrapTo180(a2 - a1));
end

function redraw(fig)
if ~ishandle(fig), return; end
ud = fig.UserData;
theta = ud.theta;
[E, F, G, D, A, B, beta_deg, alpha_deg, ok] = arm_pose(theta, ud);
h = ud.h;
thc = ud.thc;

set(h.EF, "XData", [E(1) F(1)], "YData", [E(2) F(2)])
set(h.FG, "XData", [F(1) G(1)], "YData", [F(2) G(2)])
set(h.GD, "XData", [G(1) D(1)], "YData", [G(2) D(2)])
set(h.AD, "XData", [A(1) D(1)], "YData", [A(2) D(2)])
set(h.AE, "XData", [A(1) E(1)], "YData", [A(2) E(2)])

set(h.F, "XData", F(1), "YData", F(2))
set(h.G, "XData", G(1), "YData", G(2))
set(h.D, "XData", D(1), "YData", D(2))
set(h.A, "XData", A(1), "YData", A(2))
set(h.B, "XData", B(1), "YData", B(2))

set(h.circF, "XData", F(1)+ud.l2*cos(thc), "YData", F(2)+ud.l2*sin(thc))
set(h.circD, "XData", D(1)+ud.l1*cos(thc), "YData", D(2)+ud.l1*sin(thc))
set(h.circA, "XData", A(1)+ud.lp*cos(thc), "YData", A(2)+ud.lp*sin(thc))

labels = ["E","F","G","D","A","B"];
pts = [E; F; G; D; A; B];
for i = 1:5
    set(h.txt(i), "Position", [pts(i,1)+0.35, pts(i,2)+0.35, 0], "String", labels(i))
end

if ok
    status = "ok";
else
    status = "IK FAIL";
end

set(h.alg, "String", sprintf([ ...
    "Lengths (ik.c)\n" ...
    "  EF = lb = %.2f\n" ...
    "  FG = l2 = %.2f\n" ...
    "  GD = l1 = %.2f\n" ...
    "  AD = lp = %.2f\n" ...
    "  h  = %.2f\n\n" ...
    "Pose\n" ...
    "  theta = %6.2f deg\n" ...
    "  beta  = %6.2f deg   (at F)\n" ...
    "  alpha = %6.2f deg   (at G)\n" ...
    "  IK    = %s\n\n" ...
    "Points\n" ...
    "  E = (%6.2f, %6.2f)\n" ...
    "  F = (%6.2f, %6.2f)\n" ...
    "  G = (%6.2f, %6.2f)\n" ...
    "  D = (%6.2f, %6.2f)\n" ...
    "  A = (%6.2f, %6.2f)\n" ...
    "  B = (%6.2f, %6.2f)\n\n" ...
    "Drag B on circle around A\n" ...
    "or move theta slider."], ...
    ud.lb, ud.l2, ud.l1, ud.lp, ud.h0, ...
    theta, beta_deg, alpha_deg, status, ...
    E, F, G, D, A, B));

if ishandle(ud.h.sld) && abs(ud.h.sld.Value - theta) > 1e-6
    ud.h.sld.Value = min(max(theta, ud.h.sld.Min), ud.h.sld.Max);
end
drawnow limitrate
end

function on_down()
fig = gcbf;
fig.UserData.dragging = true;
end

function on_up()
fig = gcbf;
if isempty(fig), return; end
fig.UserData.dragging = false;
end

function on_move()
fig = gcbf;
if isempty(fig) || ~isfield(fig.UserData, "dragging") || ~fig.UserData.dragging
    return
end
ax = fig.UserData.h.ax;
cp = ax.CurrentPoint;
xy = cp(1, 1:2);
A = [0, fig.UserData.h0];
v = xy - A;
if norm(v) < 1e-9
    return
end
% B on circle around A: angle from left-horizontal (= flat) -> theta
ang = atan2(v(2), v(1));
rel = wrapToPi(ang - pi);            % 0 when B is directly left of A
theta = abs(rad2deg(rel));
theta = min(max(theta, 0), fig.UserData.p.max_theta);
fig.UserData.theta = theta;
redraw(fig);
end

function on_slider(src, ~)
fig = ancestor(src, "figure");
fig.UserData.theta = src.Value;
redraw(fig);
end

function on_key(evt)
fig = gcbf;
switch upper(evt.Key)
    case {"Q", "ESCAPE"}
        on_quit();
end
end

function on_quit()
fig = gcbf;
if isempty(fig), return; end
fig.UserData.quit = true;
uirsume_safe(fig);
delete(fig);
end

function on_close(src)
uirsume_safe(src);
delete(src);
end

function uirsume_safe(fig)
try
    if ishandle(fig)
        uiresume(fig);
    end
catch
end
end

function x = wrapTo180(x)
x = mod(x + 180, 360) - 180;
end

function x = wrapToPi(x)
x = atan2(sin(x), cos(x));
end
