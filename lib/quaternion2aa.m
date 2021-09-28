
function [theta, u] = quaternion2aa(Q)
qw = Q(1);
qx = Q(2);
qy = Q(3);
qz = Q(4);
theta = 2*acos(qw);
s = sqrt(1-qw*qw);
if s == 0
    x = 0;
    y = 0;
    z=0;
else
    x = qx/s;
    y = qy/s;
    z = qz/s;
end

u = [x y z]';
