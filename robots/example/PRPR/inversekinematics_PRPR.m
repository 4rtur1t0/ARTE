
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
function q = inversekinematics_PRPR(robot, T)
a = eval(robot.DH.a);
L = a(4);

cphi = T(1,1);
sphi = T(2,1);
phi = atan2(sphi, cphi);

px = T(1, 4);
py = T(2, 4);
pz = T(3, 4);

pmx = px - L*cos(phi);
pmy = py - L*sin(phi);

q1 = pz;
q2 = atan2(pmy, pmx);
q3 = sqrt(pmx^2 + pmy^2);
q4 = phi - q2;

q = [q1 q2 q3 q4];
