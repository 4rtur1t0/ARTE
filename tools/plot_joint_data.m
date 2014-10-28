%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PLOT_JOINT_DATA(robot)
%
%   Plots the position, velocity and acceleration of each joint.
%   The movement in joint coordinates must be stored in the variables
%   robot.q_vector: joint position
%   robot.qd_vector: joint speeds
%   robot.qdd_vector: joint accelerations.
%
%   The variables are plotted in the figures specified by configuration.figure.q,
%   configuration.figure.qd and configuration.figure.qdd, for position, 
%   speed and acceleration respectively.
%   
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   25/02/2012
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
function plot_joint_data(robot)
global configuration

t = robot.time;
q = robot.q_vector;
qd = robot.qd_vector;
qdd = robot.qdd_vector;

figure(configuration.figure.q)

plot(t, q), grid, title('Position vs. time')
xlabel('time (s)'), ylabel('Position (rad, m)')
legend(make_legend('q', robot.DOF));

figure(configuration.figure.qd)
plot(t,qd), grid, title('Velocity vs. time')
xlabel('time (s)'), ylabel('Velocity (rad/s, m/s)')
legend(make_legend('qd', robot.DOF));

figure(configuration.figure.qdd)
plot(t,qdd), grid, title('Acceleration vs. time')
xlabel('time (s)'), ylabel('Acceleration (rad/s^2, m/s^2)')
legend(make_legend('qdd', robot.DOF));


%make a matrix legend of joint coordinates
function leg=make_legend(tag, DOF)

leg =[];
for i=1:DOF,
   leg=[leg; [tag sprintf('_%d',i)]]; 
end