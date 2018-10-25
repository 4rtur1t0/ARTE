% SCRIPT TEST FOR THE KINEMATIC PROBLEM FOR SERIAL ROBOTS

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

close all

fprintf('\nSimple test: try to reach T')


T = [1 0 0 0.3; 
    0   1  0  0.3;
    0 0 1    0.5;
    0  0  0   1];
%try this initial seed for the inverse kinematic
%different seeds do allow to obtain different solutions
q = [0 0 0 0 0 0 0]';
qinv = inverse_kinematics_sawyer(robot, T, q)

T_reach = directkinematic(robot, qinv)

%this difference should be low, since both matrices should represent the
%same position and orientation
%being it an iterative algorithm, you may find differences in both matrices
%depending on the parameters 
%robot.parameters.epsilonXYZ=0.01; and
%robot.parameters.epsilonQ=0.01;
'diff T-Treach'
T-T_reach


%these algorithms just return a single solution
n_solutions = 1;

%Try different configurations beware that, depending on the robot's topology
%not all the eight possible solutions will be feasible for an antropomorphic 6R robot.
q=[0.5 -0.4 -0.2 0.1 0.1 0.1 0.1]';

%draw the robot
drawrobot3d(robot, q)

%Now compute direct kinematics for this position q
T = directkinematic(robot, q)

%Set to zero if you want to see the robot transparent
robot.graphical.draw_transparent=0;

%try to look for a different solution
q = [0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
%Call the inversekinematic for this robot. All the possible solutions are
%stored at qinv. At least, one of the possible solutions should match q
qinv = inversekinematic(robot, T, q)


T_reach = directkinematic(robot, qinv)

%
'diff T-Treach'
T-T_reach

