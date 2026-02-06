%   INVERSE KINEMATICS FOR THE 4 DOF RRPR ROBOT
%   ALGEBRAIC SOLUTION
function q = inversekinematics_RRPR_algebraic(robot, T)
% algebraic method
% get some of the parameters
q= [0 0 0 0];
a = eval(robot.DH.a);
d = eval(robot.DH.d);
L1 = d(1);
L2 = a(4);
p = T(1:3, 4);
% compute phi
cphi = T(3, 2);
sphi = T(3, 1);
phi = atan2(sphi, cphi);
% compute q1
cq1=-T(2, 3);
sq1=T(1, 3);
q1=atan2(sq1, cq1);
q1_=q1+pi;
%normalize to -pi, pi
q1_=atan2(sin(q1_), cos(q1_));

[q2, q3] = solve_q23(p, phi, q1, L1, L2);
[q2_, q3_] = solve_q23(p, phi, q1_, L1, L2);

q4 = phi-q2;
q4_ = phi-q2_;
q = [q1 q1_
    q2 q2_
    q3 q3_
    q4 q4_];

function [q2, q3] = solve_q23(p, phi, q1, L1, L2)
k1= p(1)*cos(q1)+p(2)*sin(q1)-L2*cos(phi);
k2= p(3)- L1 - L2*sin(phi);

q2 = atan2(k2, k1);
q3 = norm([k1, k2]);

