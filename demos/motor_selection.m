%   SCRIPT TO FIND THE TORQUES AT EACH JOINT FOR DIFFERENT MOTION STATES OF
%   THE ARM.
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

function motor_selection

close all
global robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS SECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot pose: experiment by changing the pose while observing the different
%             torques at each joint
q=[0 0 -pi/2 0 0 0]; %rad

%maximum speeds for joint 1, 2, 3, 4, 5 and 6
%maximum_speeds=[3 4 5 5.5 6 6.3]; %rad/second
maximum_speeds=[3 3 4 5 5 5];%rad/second
%maximum acceleration/deceleration for each joint
%maximum_accels=[2 3 3.5 4 5 6]; %rad/second^2
maximum_accels=[5 5 6 7 8 9]; %rad/second^2

% time of the trapezoidal profile that the joint moves at maximum speed
time_at_constant_speed=0.4; %seconds


%load robot parameters
robot=load_robot('unimate', 'puma560');
drawrobot3d(robot, q)


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
% FINALLY, COMPUTE TORQUES FOR EACH MOTION STATE. 
% Please note that we consider that the robot is placed at a fixed position and consider
% different motion situations when we change the acceleration and speed at
% each joint. For each motion state, the inverse dynamic model returns the
% torques at each joint that would bring the robot to that motion.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
compute_inverse_dynamics(q, input_speeds, input_accels, time);


fprintf('\n\nOBSERVE THE PLOTS AND NOTE DOWN THE PEAK TORQUE, NOMINAL TORQUE AND MOTOR SPEEDS')
fprintf('\nNOW COMPUTE THE TORQUES FOR 5 DIFFERENT SELECTED MOTIONS STATES')
fprintf('\nPRESS ANY KEY TO CONTINUE...')

pause

% NOW COMPUTE THE WORST CASE CONSIDERING ONLY THE SPEED AND ACCELERATION AT
% 5 MOTIONS STATES
input_speeds = [zeros(6,1) maximum_speeds' maximum_speeds' maximum_speeds' zeros(6,1) ];
input_accels = [maximum_accels' maximum_accels' zeros(6,1) -maximum_accels' -maximum_accels' ];

compute_inverse_dynamics(q, input_speeds, input_accels, [1:5]);




%Computes a trapezoidal speed profile for every joint given maximum
%permitted accelerations and maximum joint speeds
function [input_speeds, input_accelerations, time]=build_trapezoidal_speed_profile(maximum_speeds, maximum_accels, total_time)

delta_time=0.01;

%build time vector: twice acceleration time plus time at constant speed
time = 0:delta_time:total_time;

input_speeds=[];
input_accelerations=[];

for j=1:length(maximum_speeds), 
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




%returns the values of velocity and speed corresponding to a given time
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
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function compute_inverse_dynamics(q, input_speeds, input_accels, time)
global robot

%adjust_view(robot)
torques=[];
for j=1:length(time), 
    fprintf('\nComputing time %d out of %d', j, length(time));
    % compute the torque to bring the robot instantaneously to this motion
    % state. change M=1  to add the effects of a 1kg mass load at the end effector
    M=1;
    %please note that the force due to the load acts on the z axis of
    tau=inversedynamic(robot, q, input_speeds(:,j), input_accels(:,j), [0  0 -9.81]', [M*9.81 0 0 0 0 0]');
    torques=[torques tau'];
end


%plot trapezoidal profiles
figure, hold, xlabel('time (s)'), ylabel('Input reference speeds (rad/s)')
plot(time, input_speeds(1,:), time, input_speeds(2,:), time, input_speeds(3,:),...
        time, input_speeds(4,:), time, input_speeds(5,:), time, input_speeds(6,:));
legend('Speed for joint 1 (qd1)','Speed for joint 2 (qd2)','Speed for joint 3 (qd3)',... 
   'Speed for joint 4 (qd4)','Speed for joint 5 (qd5)','Speed for joint 6 (qd6)' )
%plot trapezoidal profiles, acceleration
figure, hold, xlabel('time (s)'), ylabel('Input reference acceleration (rad/s)')
plot(time, input_accels(1,:), time, input_accels(2,:), time, input_accels(3,:),...
        time, input_accels(4,:), time, input_accels(5,:), time, input_accels(6,:));
legend('Acceleration for joint 1 (qd1)','Acceleration for joint 2 (qd2)','Acceleration for joint 3 (qd3)',... 
   'Acceleration for joint 4 (qd4)','Acceleration for joint 5 (qd5)','Acceleration for joint 6 (qd6)' )


% plot results. First, torques at each joint
figure, hold, xlabel('time (s)'), ylabel('Join Torques (N m)')
plot(time, torques(1,:), time, torques(2,:), time, torques(3,:),...
        time, torques(4,:), time, torques(5,:), time, torques(6,:));
legend('Torque for joint 1 ','Torque  for joint 2 ','Torque  for joint 3 ',... 
   'Torque  for joint 4','Torque  for joint 5 ','Torque  for joint 6 ' )

%plot torques at each motor
figure, hold, xlabel('time (s)'), ylabel('Motor Torques (N m)')
plot(time, torques(1,:)/robot.motors.G(1), time, torques(2,:)/robot.motors.G(2), time, torques(3,:)/robot.motors.G(3),...
        time, torques(4,:)/robot.motors.G(4), time, torques(5,:)/robot.motors.G(5), time, torques(6,:)/robot.motors.G(6));
legend('Torque at motor 1 ','Torque at motor 2 ','Torque at motor 3 ',... 
   'Torque at motor 4 ','Torque at motor 5 ','Torque at motor 6 ' )

%plot power needed by the motor at each time step, without considering the
%losses at the gears
figure, hold, xlabel('time (s)'), ylabel('Power needed by each motor (W)')
plot(time, torques(1,:).*input_speeds(1,:), time, torques(2,:).*input_speeds(2,:), time, torques(3,:).*input_speeds(3,:),...
        time, torques(4,:).*input_speeds(4,:), time, torques(5,:).*input_speeds(5,:), time, torques(6,:).*input_speeds(6,:));
legend('Power: motor 1','Power: motor 2','Power: motor 3',... 
   'Power: motor 4','Power: motor 5','Power: motor 6' )

%plot motor speed in rpm for each motor
figure, hold, xlabel('time (s)'), ylabel('Speed in r.p.m of every motor (rev/min)')
plot(time, robot.motors.G(1)*input_speeds(1,:)*30/pi, time, robot.motors.G(2)*input_speeds(2,:)*30/pi, time, robot.motors.G(3)*input_speeds(3,:)*30/pi,...
        time, robot.motors.G(4)*input_speeds(4,:)*30/pi, time, robot.motors.G(5)*input_speeds(5,:)*30/pi, time, robot.motors.G(6)*input_speeds(6,:)*30/pi);
legend('Speed at motor 1 (qd1*G)','Speed at motor 2 (qd2*G)','Speed at motor 3 (qd3*G)',... 
   'Speed at motor 4 (qd4*G)','Speed at motor 5 (qd5*G)','Speed at motor 6 (qd6*G)' )



%Now present results:
fprintf('\nMAIN RESULTS (referred to each motor): ')
fprintf('\n------------------------------------------------------------------------------------ ')
fprintf('\n                         Joint 1 - Joint 2 - Joint 3 - Joint 4  - Joint 5 - Joint 6: ')
fprintf('\nPeak Torque (N·m):        %.3f     %.3f     %.3f     %.3f      %.3f    %.3f ', max(abs(torques(1,:)/robot.motors.G(1))), max(abs(torques(2,:)/robot.motors.G(2))) , max(abs(torques(3,:)/robot.motors.G(3))) , max(abs(torques(4,:)/robot.motors.G(4))) , max(abs(torques(5,:)/robot.motors.G(5))) , max(abs(torques(6,:)/robot.motors.G(6))))
fprintf('\nNominal Torque (N·m):     %.3f     %.3f     %.3f     %.3f      %.3f    %.3f  ', abs(torques(1,round(length(torques)/2))/robot.motors.G(1)), abs(torques(2,round(length(torques)/2))/robot.motors.G(2)), abs(torques(3,round(length(torques)/2))/robot.motors.G(3))...
    , abs(torques(4,round(length(torques)/2))/robot.motors.G(4)), abs(torques(5,round(length(torques)/2))/robot.motors.G(5)), abs(torques(6,round(length(torques)/2))/robot.motors.G(6)))
fprintf('\nMax motor speed (r.p.m.): %.1f    %.1f    %.1f    %.1f     %.1f   %.1f  ', max(abs(robot.motors.G(1)*input_speeds(1,:))*30/pi), max(abs(robot.motors.G(2)*input_speeds(2,:))*30/pi), max(abs(robot.motors.G(3)*input_speeds(3,:))*30/pi)...
    ,max(abs(robot.motors.G(4)*input_speeds(4,:))*30/pi), max(abs(robot.motors.G(5)*input_speeds(5,:))*30/pi), max(abs(robot.motors.G(6)*input_speeds(6,:))*30/pi))
fprintf('\n------------------------------------------------------------------------------------ ')


