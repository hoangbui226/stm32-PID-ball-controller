function servo = ik_to_servo(deg, offset, p)
% IK_TO_SERVO  Match main.c: cmd = 90 - deg + offset, clamped.
% deg may be a vector of horn angles from ik_solve.

if nargin < 2 || isempty(offset)
    offset = p.servo_offset;
end

cmd = 90 - deg + offset;
cmd = min(max(cmd, p.servo_min), p.servo_max);
servo = round(cmd);
end
