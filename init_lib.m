%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   init_lib 
%   Initialization file
%   The script creates the global variables: configuration and robot
%
%   The configuration variable stores parameters related to the simulation,
%   such as:
%   configuration.delta_time: Is the step time in seconds used to move the
%       robot. The time is thus discretized in small steps of size delta_time.
%       Selecting configuration.delta_time=0.01 usually results in a fair
%       simulation. Select larger values, such as delta_time=0.1 if you
%       experience that the simulation is too slow.
%   
%   configuration.time_delay: During a simulation, the robot is drawn at different
%       positions at the Matlab's plot. For example, to simulate a trajectory, 
%       the robot is drawn with different joint values at a particular sequence.
%       After drawing the robot at a particular pose, the system waits for time_delay
%       seconds, so that the figure can be observed by the user. Depending
%       on the system, values near  configuration.time_delay=0.001 are
%       correct. Please, increase  configuration.delta_time and configuration.time_delay
%       if you are not able to visualize the simulation smoothly.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio, arturo.gil@umh.es
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
%   http://arvc.umh.es/arte
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


%presentation

echo on
%     ARTE (A Robotics Toolbox for Education)
%     Copyright (C) 2012  Arturo Gil Aparicio, arturo.gil@umh.es
%     http://arvc.umh.es/arte
%  
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU  Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with this program.  If not, see  <http://www.gnu.org/licenses/>.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%     To begin with, try loading a robot:
%     >> robot = load_robot('ABB','IRB140');
%
%     Next, draw it on its zero position:
%     >> drawrobot3d(robot,[0 0 0 0 0 0])
%
%     Next, try a different pose:
%     >> drawrobot3d(robot,[0 pi/2 -pi/2 0 0 0])
%
%     Finally, use the teach pendant to move the robot:
%     >> teach
%
%   Please, type:
%   >> configuration.delta_time=0.1 
%   if you find that the simulation is running too slow.
%
%   Please, type:
%   >> configuration.time_delay = 0.01;
%   if the simulation plot does not show the robot correctly.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
echo off
    
%initialize variables
global configuration robot

%base directory for the library
configuration.libpath = pwd();

configuration.delta_time = 0.01; %minimum time between two consecutive poses of the robot. Adjust this time from 0.01 to 0.2, according to your processor
configuration.time_delay = 0.001; %delay in seconds between frames when animating a robot. see animate.m for more information

configuration.min_resolution=0.0001;

%Number each of the figures to plot everything
configuration.figure.robot=1;
configuration.figure.q=2;
configuration.figure.qd=3;
configuration.figure.qdd=4;

%Add folders and subfolders starting from the current path
addpath(genpath(configuration.libpath))
