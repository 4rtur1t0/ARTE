% Given the normal, we must compute an orientation that is perpendicular
% to the normal
% surface sign indicates whether the Z vector of the end_effector must have
% the same direction as n or opposite.
function T = build_T_from_obstacle(obstacle, p0)
global robot
n = obstacle.n;
nx = n(1);
ny = n(2);
nz = n(3);

%this is vector z7 of the end effector
z7 = n;
x0=[1 0 0]';
%x7 points in the direction of the first line
x7 = cross(x0, n);

%y7 to form rotation matrix
y7 = cross(z7,x7);

T = zeros(4,4);
R = [x7 y7 z7];
T(1:3,1:3)=R;
T(1:3,4)=p0;
T(4,4)=1;

%constant orientation
T = eye(4);

%return T in the oposite direction!
%with robot coupling tranformation
T=T*inv(robot.Tcoupling);


