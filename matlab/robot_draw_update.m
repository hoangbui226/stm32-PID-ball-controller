function robot_draw_update(h, joints, ball, p, theta, phi)
% ROBOT_DRAW_UPDATE  Move robot artists to a new pose.

A = joints.A; B = joints.B; C = joints.C;
n = [sind(theta)*cosd(phi), sind(theta)*sind(phi), cosd(theta)];
nz = max(n(3), 1e-3);
for i = 1:3
    set(h.lower(i), "XData", [B(i,1) C(i,1)], "YData", [B(i,2) C(i,2)], "ZData", [B(i,3) C(i,3)])
    set(h.upper(i), "XData", [C(i,1) A(i,1)], "YData", [C(i,2) A(i,2)], "ZData", [C(i,3) A(i,3)])
    set(h.top(i), "XData", A(i,1), "YData", A(i,2), "ZData", A(i,3))
    set(h.base(i), "XData", B(i,1), "YData", B(i,2), "ZData", B(i,3))
end
zg = p.h - (n(1)*h.xg + n(2)*h.yg) / nz;
zg(h.xg.^2 + h.yg.^2 > p.plate_radius^2) = NaN;
set(h.plate, "ZData", zg)
z_ball = p.h - (n(1)*ball(1) + n(2)*ball(2)) / nz + 0.4;
set(h.ball, "XData", ball(1), "YData", ball(2), "ZData", z_ball)
end
