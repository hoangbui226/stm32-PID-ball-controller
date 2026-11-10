function h = robot_draw(ax, joints, ball, p, theta, phi)
% ROBOT_DRAW  Create 3D robot artists on axes ax.

if nargin < 1 || isempty(ax)
    ax = gca;
end
hold(ax, "on")
colors = lines(3);
h.lower = gobjects(3, 1);
h.upper = gobjects(3, 1);
h.top = gobjects(3, 1);
h.base = gobjects(3, 1);
for i = 1:3
    h.lower(i) = plot3(ax, nan, nan, nan, "-", "Color", colors(i,:), "LineWidth", 2.5);
    h.upper(i) = plot3(ax, nan, nan, nan, "-", "Color", colors(i,:), "LineWidth", 2.0);
    h.top(i) = plot3(ax, nan, nan, nan, "o", "Color", colors(i,:), ...
        "MarkerFaceColor", colors(i,:), "MarkerSize", 7);
    h.base(i) = plot3(ax, nan, nan, nan, "ks", "MarkerFaceColor", "k", "MarkerSize", 6);
end
[xg, yg] = meshgrid(linspace(-p.plate_radius, p.plate_radius, 12));
h.xg = xg;
h.yg = yg;
h.plate = surf(ax, xg, yg, zeros(size(xg)), ...
    "FaceAlpha", 0.35, "EdgeColor", [0.7 0.7 0.7]);
h.ball = plot3(ax, nan, nan, nan, "ro", "MarkerFaceColor", "r", "MarkerSize", 12);
axis(ax, "equal")
grid(ax, "on")
xlabel(ax, "x (cm)"); ylabel(ax, "y (cm)"); zlabel(ax, "z (cm)")
xlim(ax, [-10 10]); ylim(ax, [-10 10]); zlim(ax, [0 12])
view(ax, 35, 22)

if nargin >= 5 && ~isempty(joints)
    robot_draw_update(h, joints, ball, p, theta, phi)
end
end
