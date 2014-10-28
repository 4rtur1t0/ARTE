%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MoveL: Make a linear planning in space
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
function MoveL(robtarget, speeddata, zonedata, gripper, Wobj)

global configuration robot

fprintf('\nCall to MoveL', robot.name);

%obtain current joint coordinates
q_current= robot.q;

%Ttool, transformation from the robot's end to the TCP
Ttool = transform_to_homogeneous(gripper(2:end));

%compute current transformation matrix, as well as position p0 and
%quaternion q0
%current position and orientation of the last link
T=directkinematic(robot, robot.q);

%consider the total transformation including the gripper
Ttotal = T*Ttool;

%last target
p0 = Ttotal(1:3,4);
%orientation
Q0 = T2quaternion(Ttotal);


%in this case, plan a linear trajectory from point
%if robot.last_zone_data=='fine'
%compute target joint coordinates, from the inverse kinematics
%select desired joint values by using conf data from robtarget
%select configuration
final_conf = get_conf_data(robtarget);
Ttotal = transform_to_homogeneous(robtarget);


%final position
p1 = Ttotal(1:3,4);
% and orientation
Q1 = T2quaternion(Ttotal);

distance = norm(p1-p0);
%unit vector in the direction of the trajectory normalize
u = (p1-p0)/norm(p1-p0);

speed = obtain_linear_speed(robot, speeddata);

tmax = distance/speed;

%compute current robot speed in modulus. m/s
v_current = 1;

%in this case, if the total time for the movement is lower than the
%configuration.delta_time time then 10 points are interpolated
if tmax <= configuration.delta_time
    %disp('/RAPID/MOVEL: No movement is performed. Making a null movement of 1 second');
    %tmax = 2*configuration.delta_time; %avoid errors 
    %distance = 1;
    %u = (p1-p0)/norm(p1-p0);
    t = [0:tmax/10:tmax]';
else
    %local time for the planning, normal case
    t = [0:configuration.delta_time:tmax]';
end


%when fine is specified, the radius is zero, otherwise,
%the radius represents a sphere around the target where the
%movement is changed to the next target
radius = obtain_zone_data(zonedata);
if radius == 0 %target point is fine
    %plan an acceleration profile one the line that connects both
    %points. Asume as a one joint planifier
    [pos, vel, accel] = single_joint_spline(0, distance, v_current, 0, t);
else
    [pos, vel, accel] = single_joint_spline(0, distance, v_current, speed, t);
end

%current_conf = compute_configuration(robot, robot.q);    

pp=[];
q_line=[];

N=length(pos);
for j=1:N,
    pj = p0 + u*pos(j);
    
    %interpolate between the initial and final quaternions
    %to find an orientation between both
    [Qm] = slerp(Q0, Q1, (j-1)/N, 0.01);
    
    %find homogeneous matrix corresponding to pi and Qm
    Ttotal = quaternion2T(Qm, pj);
    
    pp=[pp T(1:3,4)];
    
    %find T, of the robot's end points
    %In case there is a tool attached to the robot, undo the Tool
    %transformation
    %T=Ttotal*inv(Ttool);
    T=Ttotal/Ttool;
    
    %find all the possible solutions to reach T
    qinv = inversekinematic(robot, T);
    
    % select at each timestep the joint coordinates closer to the current
    % one. This strategy fails if the robot goes through a singularity.
    % In addition, care must be taken when programming the robot, since the
    % configuration control may stop the robot. In RAPID the ConfJ and
    % ConfL instructions should be used to control the possible changes of
    % configuration in linear or joint coordinated motions.
    q=select_closest_joint_coordinates(qinv, robot.q);
     
    %store the joint coordinates in a vector for ulterior animation
    q_line = [q_line q];
    
    %the robot performs the movement until the index found. The coordinates, joint speed and acceleratin
    %are stored and used in the planning of the next point
    robot.q=q;
    robot.qd=zeros(robot.DOF,1);
    robot.qdd=zeros(robot.DOF,1);
    
    %store all the trajectory for plotting
    %the joint trajectories, speeds and acceleration of susequent movements are
    %store here
    robot.q_vector=[robot.q_vector q];
    robot.qd_vector=[robot.qd_vector zeros(robot.DOF,1)];
    robot.qdd_vector=[robot.qdd_vector zeros(robot.DOF,1)];
    
    %update current configuration
   % current_conf = compute_configuration(robot, robot.q);  
end

  
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
animate(robot, q_line);

%Plot an arroy between p0 an p1
vect_arrow(p0,p1,'b')

plot3(pp(1,:),pp(2,:),pp(3,:), 'k', 'LineWidth', 2)


