%   CODE TO COMPUTE THE TORQUES AT EACH JOINT FOR DIFFERENT MOTION STATES OF
%   a 2 DOF robotic arm. SHOULD BE COMPLETED BY THE STUDENT
%   
%   In order to select an actuator (i.e. an electric motor, brushless for
%   example), we may need.
%   Nominal torque and speed: torque and speed for the 80% of the use of the motor. 
%   Peak torque and speed: torque and speed for short periods of time. That
%   is: a higher torque that can be exerted at higher speeds during a
%   maximum of a 20% of the time.
%   
%   Of course, the 80-20% are just bare numbers and should be given by the
%   robot manufacturer. Actually the torque in any motor depends directly
%   on the quantity of current that can be driven into the motor.
%   Typically, the peak torque is associated with a peak current. If the
%   peak current is maintained for a long time the motor will not be able
%   to dissipate the heat inside, thus generating high temperatures that
%   could melt the coils, conductors... etc.
%
%   The script uses the inverse dynamic model of the robot to simulate
%   different motion states and compute the torques for each situation.
%   The torques at each joint, as well as the torques at each motor are computed
%   (reduced by the gear ratio). A trapezoidal speed profile can be
%   selected at the parameters section of this file. This trapezoidal
%   profile should meet the desired features of the robot, such as maximum
%   joint speed, maximum acceleration/deceleration.
%   Please note that the movement of the robot is not simulated. The reader
%   should imagine that the manipulator is fixed at a given initial
%   position and the inverse dynamic function returns the torques at each
%   joint necessary to bring the arm to that motion state (defined by position q,
%   speed qd and acceleration qdd).
%   
%   As a reference, below you can find the call to the inverse dynamics
%   function.
% 
%   TAU = inversedynamic(robot, Q, QD, QDD, GRAV, FEXT)
%
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

function motor_selection_2dofplanar

close all
global robot
robot.dynamics.friction = 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS SECTION
%   Feel free to change the values of q, maximum_speeds and 
%   maximum_accels. 
%  
%   This script tries to allow the student to test any mechanism
%   at the worst case. In this sense, q should be adjusted 
%   as the pose where each joint would (statically) be needing a
%   higher torque.
%   The maximum_speeds and maximum_acceleration define a trapezoidal
%   speed profile. This trapezoidal speed is used by most machines
%   to command changes in speed in any of their joints.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot pose: experiment by changing the pose while observing the different
%             torques at each joint
q=[0 0]; %rad
%maximum speeds at for each joint
maximum_speeds=[pi pi];%rad/second
%maximum acceleration/deceleration for each joint
maximum_accels=[pi/1 pi/1]; %rad/second^2

% time of the trapezoidal profile that the joint moves at maximum speed
time_at_constant_speed=3; %seconds

%load robot parameters. Just uncomment this line
robot=load_robot('example', '2dofplanar');
drawrobot3d(robot, q)

% CUIDADO: los resultados se calculan con estas ratios.
robot.motors.G = [20 10]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  FIRST, COMPUTE TRAPEZOIDAL PROFILES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%compute acceleration plus deceleration times for every joint
time_acc = 2*maximum_speeds./maximum_accels+time_at_constant_speed;

%compute the total time for the slowest joint
total_time=max(time_acc);

% Trapezoidal speed profiles for each joint
[input_speeds, input_accels, time]=build_trapezoidal_speed_profile(maximum_speeds, maximum_accels, total_time);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FINALMENTE, CALCULE LOS PARES PARA CADA ESTADO DE MOVIMIENTO.
% TENGA EN CUENTA QUE 
% ESCRIBA UNA FUNCIÓN COMO LA SIGUIENTE
% compute_inverse_dynamics(q, input_speeds, input_accels, time);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute_inverse_dynamics(q, input_speeds, input_accels, time);









%Computes a trapezoidal speed profile for every joint given maximum
%permitted accelerations and maximum joint speeds
function [input_speeds, input_accelerations, time]=build_trapezoidal_speed_profile(maximum_speeds, maximum_accels, total_time)
delta_time=0.01;

%build time vector: twice acceleration time plus time at constant speed
time = 0:delta_time:total_time;

input_speeds=[];
input_accelerations=[];

for j=1:length(maximum_speeds) 
    vel_row=[];
    acc_row=[];
    for i=1:length(time), 
        [vel acc] = compute_values(time(i), maximum_speeds(j), maximum_accels(j), total_time);
        vel_row = [vel_row vel];
        acc_row = [acc_row acc];        
    end
    input_speeds = [input_speeds; vel_row];
    input_accelerations = [input_accelerations; acc_row];    
end


% returns the values of velocity and speed corresponding to a given time
function [vel acc]=compute_values(time_i, vel_max, acc_max, total_time)
tacc = vel_max/acc_max;
tdec = total_time-tacc;

if time_i < tacc
    vel = time_i.*acc_max;
    acc = acc_max;
    return;
elseif (time_i >= tacc) & (time_i < tdec)
    vel = vel_max;
    acc = 0;
    return;
else % time_i> tdec
    vel = vel_max-(time_i-tdec)*acc_max;
    acc = -acc_max;    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   COMPUTE THE INVERSE DYNAMICS FOR EACH MOTION STATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function compute_inverse_dynamics(q, input_speeds, input_accels, time)
global robot


