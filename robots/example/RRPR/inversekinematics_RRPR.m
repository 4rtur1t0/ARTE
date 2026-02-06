
%   INVERSE KINEMATICS FOR THE 4 DOF PLANAR ROBOT
%   JUST USING A GENERIC gradient descent algorithm based on the
%   Moore-Penrose pseudo-inverse
%
%   Solves the inverse kinematic problem in various situations.
%   A Jacobian based method is used.
%   The method tries to reach the given position/orientation while, at the 
%   same time, maximizing/minimizing a secondary target.
%
%   e.g. reach position/orientation and maximize manipulability det(J'J)
%   
%   q0: starting initial solution
%   Tf--> final position/orientation wanted as a homogeneous matrix
function q = inversekinematics_RRPR(robot, T)
% geometric method
% get some of the parameters
q= [0 0 0 0];
a = eval(robot.DH.a);
d = eval(robot.DH.d);
L1 = d(1);
L2 = a(4);
% obtain the end effector's target p and the vector x4
p = T(1:3, 4);
x4 = T(1:3, 1);
% pseudo wrist
pm = p - L2*x4;
% compute q1
q1=atan2(pm(2), pm(1));
% alternate solution for q1
q1_ = q1 + pi
q1_= atan2(sin(q1_), cos(q1_))
% change coordinates of pm to system 1
A01 = dh(robot, [q1, 0, 0, 0], 1);
pm_ = inv(A01)*[pm;1];
% solve for q2
q2 = atan2(pm_(2), pm_(1));
%alternate solution for q2
A01_ = dh(robot, [q1_, 0, 0, 0], 1);
pm_ = inv(A01_)*[pm;1];
% solve for q2
q2_ = atan2(pm_(2), pm_(1));
% just a single solution for q3
q3 = norm(pm_(1:3));

% compute now phi
x1 = A01(1:3, 1);
y1 = A01(1:3, 2);
phi = atan2(y1'*x4, x1'*x4);
q4 = phi-q2;
q4_ = phi-q2_;
q = [q1 q1_
    q2 q2_
    q3 q3
    q4 q4_];
