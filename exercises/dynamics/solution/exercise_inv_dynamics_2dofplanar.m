%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 	TEST THE DYNAMICS OF A 2DOF robot under different circumstances.
%   
%   Do exercises 1, 2 and 3
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2016, by Arturo Gil Aparicio
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
function exercise_inv_dynamics_2dofplanar()
close all

%Just load the robot once
robot=load_robot('example', '2dofplanar');

%TODO: uncommment as you solve the exercises%
%exercise1(robot)%
%exercise2(robot)
exercise3(robot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Given the following specifications, find the maximum torques if:
%   
%   g = [0 -9.81 0]'
%   amax = [2 2] rad/s^2, max angular acceleration
%   wmax = [3 3], max angular speed
%   
%   OBTAIN tau for different cases and find the maximum value.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise1(robot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TODO: CHANGE q to find the "worst" pose
q=[pi/2 pi/2]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qd=[2 2]'; %speed --> does it make any difference?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: CHANGE qdd [-3 3] to find whether higher torques are achieved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qdd = [3 3]'; %--> max acceleration

g=[0 -9.81 0]'; % acting on the Y0 axis.

fext = [0 0 0 0 0 0]';

figure, drawrobot3d(robot, q)

%Compute the torques
%TODO: store the results and find the maximum torque
tau = inversedynamic(robot, q, qd, qdd, g, fext)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Given the following specifications, find the maximum torques if:
%   
%   g = [0 -9.81 0]'
%   amax = [2 2] rad/s^2, max angular acceleration
%   wmax = [3 3], max angular speed
%
%   The robot carries a payload with m=2kg at the end effector.
%   
%   OBTAIN tau for different cases and find the maximum value.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise2(robot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TODO: CHANGE q to find the "worst" pose
q=[0 0]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qd=[0 0]'; %speed --> does it make any difference?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: CHANGE qdd [-3 3] to find whether higher torques are achieved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qdd = [0 0]'; %--> max acceleration

m = 2;          %kg
g=[0 -9.81 0]'; % acting on the Y0 axis.

%forces and moments are expressed in the base reference frame
fext = [ m*g'  0 0 0]';

figure, drawrobot3d(robot, q)

%Compute the torques
tau = inversedynamic(robot, q, qd, qdd, g, fext)

%store the results and find the maximum torque



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Given the following specifications, find the maximum torques if the robot 
%   follows a trajectory from points in cartesian coordinates:
%   
%   initial = (1.5, 0.2)
%   final = (0.2,1.3)
%   
%   assume that acceleration stays constant at:    
%   
%       amax = [2 2] rad/s^2, max angular acceleration
%       wmax = [3 3], max angular speed
%   The robot carries a piece at the end effector with mass=2kg.
%   g = [0 -9.81 0]'
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise3(robot)
robot.q = [0 0]
p0 = [1.5, 0.2]; %initial point
p1 = [0.2  1.3]
speed = 0.5; %m/s, linear speed

distance = norm(p1-p0);
%unit vector in the direction of the trajectory normalized
u = (p1-p0)/norm(p1-p0);
dist= linspace(0,distance,100);

%initialize T
T=eye(3);
T(1:2,4) = p0';
q_line = []
%Generate a number of 
for j = 1:length(dist),
    %find all the possible solutions to reach T
    qinv = inversekinematic(robot, T);
    
    %Select the closest coordinates from the two possible solutions of the
    %inverse kinematics.
    q=select_closest_joint_coordinates(qinv, robot.q);
     
    %store the joint coordinates in a vector for ulterior animation
    q_line = [q_line q];
    robot.q=q;
    
    pj = p0 + u*dist(j);
    T(1:2,4) = pj';    
end

q=[0 0]';
qd=[3 3]'; %speed --> does it make any difference?
qdd=[2 2]'; %--> max acceleration

m = 2;          %kg
g=[0 -9.81 0]'; % acting on the Y0 axis.

%forces and moments are expressed in the base reference frame
fext = [ m*g'  0 0 0]';

figure, drawrobot3d(robot, q)
taus=[]
for i = 1:length(q_line),
    %Compute the torques
    tau = inversedynamic(robot, q_line(:,i), qd, qdd, g, fext);
    taus = [taus tau];
end

figure, plot(q_line(1,:)), hold on
plot(q_line(2,:)), xlabel('time step'), title('Joint coordinates for a line in space')
legend('q_1', 'q_2')

figure, plot(taus(1,:)), hold on
plot(taus(2,:)), xlabel('time step'), title('Torques when the robot follows a line in space')
legend('\tau_1', '\tau_2')
