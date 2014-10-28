%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_STANFORD(robot, T)	
%   Solves the inverse kinematic problem for the STANFORD robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_STANFORD returns 4 possible solutions, thus,
%   Q is a 6x4 matrix where each column stores 6 feasible joint values. Of
%   the more general 8 possible solutions, only the 4 that are physically
%   realizable are returned. This is a consequence of the third
%   translational axis that, because of its construction, does not allow
%   negative displacements.
%
%   
%   Bibliography: The algorithm has been implemented as is and taken
%        from: "ROBOT ANALYSIS. The mechanics of Serial and Parallel
%        manipulators". Lung Weng Tsai. John Wiley and Sons, inc. ISBN:
%        0-471-32593-7. pages: 104--109.
%
%   Example code:
%
%   robot=load_robot('example', 'stanford');
%   q = [0.1 0.1 0.1 0.1 0.1 0.1];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:8,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%	See also DIRECTKINEMATIC.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function q = inversekinematic_stanford(robot, T)

%initialize, eight possible solutions
q=zeros(6,4);

%evaluate robot parameters
d = eval(robot.DH.d);

h=d(6);
g=d(2);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Q=T(1:3,4);
%W is the third orientation vector
W = T(1:3,3);
U = T(1:3,1);

%Compute wrist center position P
P = Q - h*W; 

%First compute d3
d3 = sqrt(P(1)^2+P(2)^2+(P(3)-d(1))^2-g^2);

if ~isreal(d3)
    disp('\nrobots/stanford/inversekinematic_stanford: THE END POINT IS NOT REACHABLE, IMAGINARY SOLUTION');
end

%solve for theta2, if theta2 is a solution, -theta2 is also a solution
theta2=real(-asin((P(3)-d(1))/d3)+pi/2);


%returns two possible solutions for theta1
[theta1_1, theta1_2]=solve_for_theta1(g, theta2, d3, P);


%arrange all possible solutions so far
q = [theta1_1   theta1_2;
     theta2     -theta2;
     d3         d3;
     0          0;
     0          0;
     0          0];

%solve for the last three joints
%given each of the latter solutions for (theta1, theta2),
%two possible solutions are feasible, namely, wrist up and
% wrist up
q1_1 = solve_for_last_three_joints(robot, q(:,1), T, 1);
q1_2 = solve_for_last_three_joints(robot, q(:,1), T, -1);

q2_1 = solve_for_last_three_joints(robot, q(:,2), T, 1);
q2_2 = solve_for_last_three_joints(robot, q(:,2), T, -1);
 
 
%arrange all possible solutions so far
%now, there are 4 possible solutions
q = [theta1_1  theta1_1     theta1_2  theta1_2;
     theta2    theta2       -theta2    -theta2;
     d3        d3             d3        d3  ;
     q1_1(4)   q1_2(4)       q2_1(4)   q2_2(4);
     q1_1(5)   q1_2(5)       q2_1(5)   q2_2(5) ;
     q1_1(6)   q1_2(6)       q2_1(6)   q2_2(6)];
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for first joint theta1, given theta2 and d3
% a single solution for theta1 is possible
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta1_1, theta1_2]=solve_for_theta1(g, theta2, d3, P)

H = sqrt(P(1)^2+P(2)^2);
gamma = acos(g/H);
alf = atan2(P(2),P(1));
theta1_1 = alf+gamma-pi/2;

theta1_2 = alf-gamma-pi/2;


% Solve for the last three joints asuming an spherical wrist
function q = solve_for_last_three_joints(robot, q, T, wrist)
%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Z=T(1:3,3);
X=T(1:3,1);	% X orientation vector of the end effector


% Obtain the position and orientation of the system 3
% using the already computed joints q1, q2 and q3
T01=dh(robot, q, 1);
T12=dh(robot, q, 2);
T23=dh(robot, q, 3);
T03=T01*T12*T23;
 
vx3=T03(1:3,1);
vy3=T03(1:3,2);
vz3=T03(1:3,3);


%find z4 normal to the plane formed by z3 and a
z4=cross(vz3, Z);	% end effector's vector a: T(1:3,3)
z4=z4/norm(z4); %normalize

if (sum(z4)==0) | (z4==NaN)
   %degenerate case
   %if this is the case, a DOF is lost, we choose arbitrarily q4=0
   % and later compute q6 to find a suitable solution for the orientation
    q(4)=0;
else
    cosq4=wrist*dot(z4,vy3);
    sinq4=wrist*dot(z4,-vx3);
    q(4)=atan2(sinq4, cosq4);
end

% solve for q5
T34=dh(robot, q, 4);
T04=T03*T34;
vx4=T04(1:3,1);
vy4=T04(1:3,2);
%find q5 as the angle formed by Z in the plane of x4 and y4
% Z is coincident with z5
cosq5=dot(Z,-vy4);	
sinq5=dot(Z,vx4);	
q(5)=atan2(sinq5, cosq5);
 
% solve for q6 
T45=dh(robot, q, 5);
T05=T04*T45;
vx5=T05(1:3,1);
vy5=T05(1:3,2);
cosq6=dot(X,vx5);
sinq6=dot(X,vy5);	% Vector de orientaci�n n: T(1:3,1)
q(6)=atan2(sinq6, cosq6);
 
