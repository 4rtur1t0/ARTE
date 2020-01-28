%   SCRIPT TO COMPUTE THE TORQUES AT EACH JOINT FOR DIFFERENT MOTION STATES OF
%   a 2 DOF robotic arm.
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
maximum_speeds=[-pi pi];%rad/second
%maximum acceleration/deceleration for each joint
maximum_accels=[-pi/4 pi/4]; %rad/second^2

% time of the trapezoidal profile that the joint moves at maximum speed
time_at_constant_speed=2; %seconds


%load robot parameters. Just uncomment this line
robot=load_robot('example', '2dofplanar');
drawrobot3d(robot, q)

% CUIDADO: los resultados se calculan con estas ratios.
robot.motors.G = [1 1]


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
%fprintf('\nPRESS ANY KEY TO CONTINUE...')

%pause

ndof = robot.DOF
% NOW COMPUTE THE WORST CASE CONSIDERING ONLY THE SPEED AND ACCELERATION AT
% 5 MOTIONS STATES
input_speeds = [zeros(ndof,1) maximum_speeds' maximum_speeds' maximum_speeds' zeros(ndof,1) ];
input_accels = [maximum_accels' maximum_accels' zeros(ndof,1) -maximum_accels' -maximum_accels' ];

%compute_inverse_dynamics(q, input_speeds, input_accels, [1:5]);




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
for j=1:length(time) 
    fprintf('\nComputing time %d out of %d', j, length(time));
    % compute the torque to bring the robot instantaneously to this motion
    % state. change M=1  to add the effects of a 1kg mass load at the end effector
    M=0.1;
    %please note that the force due to the load acts on the z axis of
    tau=inversedynamic(robot, q, input_speeds(:,j), input_accels(:,j), [0  -9.81 0]', [0 -M*9.81 0 0 0 0]');
    torques=[torques tau];
end


%plot trapezoidal profiles
figure, hold, xlabel('time (s)'), ylabel('Input reference speeds (rad/s)')
plot(time, input_speeds(1,:), time, input_speeds(2,:));
legend('Speed for joint 1 (qd1)','Speed for joint 2 (qd2)')
%plot trapezoidal profiles, acceleration
figure, hold, xlabel('time (s)'), ylabel('Input reference acceleration (rad/s)')
plot(time, input_accels(1,:), time, input_accels(2,:));
legend('Acceleration for joint 1 (qd1)','Acceleration for joint 2 (qd2)')


% plot results. First, torques at each joint
figure, hold, xlabel('time (s)'), ylabel('Join Torques (N m)')
plot(time, torques(1,:), time, torques(2,:));
legend('Torque for joint 1 ','Torque  for joint 2 ')

%plot torques at each motor
figure, hold, xlabel('time (s)'), ylabel('Motor Torques (N m)')
plot(time, torques(1,:)/robot.motors.G(1), time, torques(2,:)/robot.motors.G(2));
legend('Torque at motor 1 ','Torque at motor 2 ' )

%plot power needed by the motor at each time step, without considering the
%losses at the gears
figure, hold, xlabel('time (s)'), ylabel('Power needed by each motor (W)')
plot(time, torques(1,:).*input_speeds(1,:), time, torques(2,:).*input_speeds(2,:));
legend('Power: motor 1','Power: motor 2' )

%plot motor speed in rpm for each motor
figure, hold, xlabel('time (s)'), ylabel('Speed in r.p.m of every motor (rev/min)')
plot(time, robot.motors.G(1)*input_speeds(1,:)*30/pi, time, robot.motors.G(2)*input_speeds(2,:)*30/pi);
legend('Speed at motor 1 (qd1*G)','Speed at motor 2 (qd2*G)' )



%Now present results:
fprintf('\nMAIN RESULTS (referred to each motor): ')
fprintf('\n------------------------------------------------------------------------------------ ')
fprintf('\n                         Joint 1 - Joint 2 ')
fprintf('\nPeak Torque (Nm):        %.3f     %.3f    ', max(abs(torques(1,:)/robot.motors.G(1))), max(abs(torques(2,:)/robot.motors.G(2))))
fprintf('\nNominal Torque (Nm):     %.3f     %.3f     ', abs(torques(1,round(length(torques)/2))/robot.motors.G(1)), abs(torques(2,round(length(torques)/2))/robot.motors.G(2)))
fprintf('\nMax motor speed (r.p.m.): %.1f    %.1f   ', max(abs(robot.motors.G(1)*input_speeds(1,:))*30/pi), max(abs(robot.motors.G(2)*input_speeds(2,:))*30/pi))
fprintf('\n------------------------------------------------------------------------------------ ')


