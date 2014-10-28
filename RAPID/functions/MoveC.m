%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MoveC: Make a circular path in space.
%   The circular path is defined by three points:
%   - The current robot position.
%   - Robtarget1, which should be traversed by the robot.
%   - Robtarget2, as the final point.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function MoveC(robtarget1, robtarget2, speeddata, zonedata, gripper, Wobj)

global configuration robot

fprintf('\nCall to MoveC', robot.name);


%Ttool, transformation from the robot's end to the TCP
Ttool = transform_to_homogeneous(gripper(2:end));

%current position and orientation
T=directkinematic(robot, robot.q);

Ttotal = T*Ttool;

%current position and orientation
p1 = Ttotal(1:3,4);
%orientation
Q1 = T2quaternion(Ttotal);

Ttotal = transform_to_homogeneous(robtarget1);
%middle, through point
p2 = Ttotal(1:3,4);
% and orientation
Q2 = T2quaternion(Ttotal);

%final position
Ttotal = transform_to_homogeneous(robtarget2);
%middle, through point
p3 = Ttotal(1:3,4);
% and orientation
Q3 = T2quaternion(Ttotal);

%fit a circle with three points
[pm, R, v1, v2] = circlefit3d(p1', p2', p3');

h=figure(configuration.figure.robot);

speed = obtain_linear_speed(robot, speeddata);
distance = 2*pi*R/2;
tmax = distance/speed;

if tmax <= configuration.delta_time
    disp('/RAPID/MOVEC: No movement is performed. Making a null movement of 1 second');
    tmax = 2*configuration.delta_time; %avoid errors 
    distance = 1;
    u=[0 0 0]';
end


%vectors joining the central point, starting and end point
x1 = (p1-pm')/norm(p1-pm');
x2 = (p2-pm')/norm(p2-pm');
x3 = (p3-pm')/norm(p3-pm');

theta_ini=compute_angle(v1, v2, x1);
theta_middle=compute_angle(v1, v2, x2);
theta_end=compute_angle(v1, v2, x3);


delta_theta=speed/R*configuration.delta_time;


theta=[theta_ini:delta_theta:theta_middle theta_middle:delta_theta:theta_end];

current_conf = compute_configuration(robot, robot.q);    

q_circle=[];
pp=[];
N=length(theta);
for j=1:N,
   
    pj  = pm' + v1'*R*cos(theta(j)) + v2'*R*sin(theta(j));
    
    pp = [pp pj];
    
    %interpolate between the initial and final quaternions
    %to find an orientation between both
    [Qm] = slerp(Q1, Q3, (j-1)/N, 0.01);
    
    %find homogeneous matrix corresponding to pi and Qm
    Ttotal = quaternion2T(Qm, pj);
    
    T=Ttotal*inv(Ttool);
    
    %find all the possible solutions to reach T
    qinv = inversekinematic(robot, T);
    
    %interpolate configuration
    %middle_conf = current_conf*(1-(i-1)/N) + final_conf*((i-1)/N);
    %select at each timestep the closes configuration
   % q=select_closest_configuration(robot, qinv, middle_conf);
    %q=select_closest_configuration(robot, qinv, current_conf);
     q=select_closest_joint_coordinates(qinv, robot.q);
   
    %store the joint coordinates in a vector for ulterior animation
    q_circle = [q_circle q];
    
    
    %the robot performs the movement until the index found. The coordinates, joint speed and acceleratin
    %are stored and used in the planning of the next point
    robot.q=q;
    robot.qd=zeros(robot.DOF,1);
    robot.qdd=zeros(robot.DOF,1);
    
    %store all the trajectory for plotting
    %the joint trajectories, speeds and acceleration of susequent movements are
    %store here
  %  robot.q_vector=[robot.q_vector q];
  %  robot.qd_vector=[robot.qd_vector zeros(robot.DOF,1)];
  %  robot.qdd_vector=[robot.qdd_vector zeros(robot.DOF,1)];
  
     robot.q_vector=[robot.q_vector q];
    robot.qd_vector=[robot.q_vector zeros(robot.DOF,1)];
    robot.qdd_vector=[robot.q_vector zeros(robot.DOF,1)];
  
    %update current configuration
    current_conf = compute_configuration(robot, robot.q);  
end


%local time for the planning
t = [0:configuration.delta_time:2*tmax]';

t=t(1:N);
  
%a global time for the planning is computed.
%in this way, the total trajectory of different movements can be plotted
if length(robot.time)==0
    tend = 0;
else
    tend = robot.time(end);
end
t = t + tend;
%store total time
robot.time=[robot.time t'];

%Test whether there are joints outside mechanical limits
test_joint_limits(robot);

%Plot position, velocity and acceleration
%plot_joint_data(robot);
%Now, animate the robot in 3D
animate(robot, q_circle);

%Plot an arroy between pm an p1, p2 and p3
vect_arrow(pm', p1,'r')
vect_arrow(pm', p2,'g')
vect_arrow(pm', p3,'b')
%Plot the trajectory
plot3(pp(1,:),pp(2,:),pp(3,:), 'k', 'LineWidth', 2)




%compute the angle between x and v1 in the reference system formed by v1
%and v2
function theta=compute_angle(v1, v2, x)
%cos
ctheta = dot(v1, x);
stheta = dot(v2, x);

theta = atan2(stheta, ctheta);


