%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_tx90(robot, T)	
%   Solves the inverse kinematic problem for the STAUBLI TX90 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_TX returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   Example code:
%
%   robot=load_robot('staubli', 'rx160l');
%   q = [0 0 0 0 0 0];	
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
function q = inversekinematic_tx90(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);


%See geometry at the reference for this robot
L6=abs(d(6));

A1 = a(1);


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]' - L6*W

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi +2*betaq1 is also a solution
% r : line between origen of frame zero and the projection of the wrist Pm on
% the plane x0-y0
% gammaq1: angle between x0 and projection of x1 on plane x0-y0
% betaq1 : angle between  projection of x1 on plane x0-y0, and r.

r=sqrt(Pm(1)^2 + Pm(2)^2);
betaq1 = asin(0.05/r);
gammaq1=atan2(Pm(2),Pm(1));
q1 = gammaq1 - betaq1;




%
%solve for q2
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);
q2_2=solve_for_theta2(robot, [q1+pi+2*betaq1 0 0 0 0 0 0], Pm);

%solve for q3
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);
q3_2=solve_for_theta3(robot, [q1+pi+2*betaq1 0 0 0 0 0 0], Pm);


%Arrange solutions, there are 8 possible solutions so far.
% if q1 is a solution, q1* = q1 + pi is also a solution.
% For each (q1, q1*) there are two possible solutions
% for q2 and q3 (namely, elbow up and elbow up solutions)
% So far, we have 4 possible solutions. Howefer, for each triplet (theta1, theta2, theta3),
% there exist two more possible solutions for the last three joints, generally
% called wrist up and wrist down solutions. For this reason, 
%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1         q1         q1        q1       q1+pi+2*betaq1   q1+pi+2*betaq1   q1+pi+2*betaq1   q1+pi+betaq1;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];

%leave only the real part of the solutions
q=real(q);


%normalize q to [-pi, pi]
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));


% solve for the last three joints
% for any of the possible combinations (theta1, theta2, theta3)
for i=1:2:size(q,2),
    qtemp = solve_spherical_wrist_tx90xl(robot, q(:,i), T, 1); %wrist up
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    
    qtemp = solve_spherical_wrist_tx90xl(robot, q(:,i), T, -1); %wrist down
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
L2=abs(a(2));
L3=abs(d(4));

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma =(acos((L2^2+r^2-L3^2)/(2*r*L2)));

if ~isreal(gamma)
    disp('WARNING:inversekinematic_tx90: the point is not reachable for this configuration, imaginary solutions'); 
    %gamma = real(gamma);
end

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) = pi/2 - beta - gamma; %elbow up
q2(2) = pi/2 - beta + gamma; %elbow down


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
L2=abs(a(2));
L3=abs(d(4));

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta =(acos((L2^2 + L3^2 - r^2)/(2*L2*L3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_tx90: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = pi - eta;
q3(2) = eta - pi;

% Solve the special case of this spherical wrist
% For wrists that whose reference systems have been placed as in the
% ABB IRB 140--> use solve_spherical_wrist2
% For wrists with the same orientation as in the KUKA KR30_jet
%--> use solve_spherical_wrist
function q = solve_spherical_wrist_tx90xl(robot, q, T, wrist)


% T is the noa matrix defining the position/orientation of the end
% effector's reference system
vx6=T(1:3,1);
vz5=T(1:3,3); % The vector a z6=T(1:3,3) is coincident with z5

% Obtain the position and orientation of the system 3
% using the already computed joints q1, q2 and q3
T01=dh(robot, q, 1);
T12=dh(robot, q, 2);
T23=dh(robot, q, 3);
T03=T01*T12*T23;

vx3=T03(1:3,1);
vy3=T03(1:3,2);
vz3=T03(1:3,3);

% find z4 normal to the plane formed by z3 and a
vz4=cross(vz3, vz5);	% end effector's vector a: T(1:3,3)

% in case of degenerate solution,
% when vz3 and vz6 are parallel--> then z4=0 0 0, choose q(4)=0 as solution
if norm(vz4) <= 0.00000001
    if wrist == 1 %wrist up
        q(4)=0;
    else
        q(4)=-pi; %wrist down
    end
else
    %this is the normal and most frequent solution
    cosq4=wrist*dot(vy3,vz4);
    sinq4=wrist*dot(-vx3,vz4);
    q(4)=atan2(sinq4, cosq4);
end
%propagate the value of q(4) to compute the system 4
T34=dh(robot, q, 4);
T04=T03*T34;
vx4=T04(1:3,1);
vy4=T04(1:3,2);

% solve for q5
cosq5=dot(-vy4,vz5);
sinq5=dot(vx4,vz5);
q(5)=atan2(sinq5, cosq5);

%propagate now q(5) to compute T05
T45=dh(robot, q, 5);
T05=T04*T45;
vx5=T05(1:3,1);
vy5=T05(1:3,2);

% solve for q6
cosq6=dot(vx6,vx5);
sinq6=dot(vx6,vy5);
q(6)=atan2(sinq6, cosq6);



