% SCRIPT TEST FOR THE KINEMATIC PROBLEM FOR THE SAWYER ROBOT
% LOAD THE SAWYER ROBOT. SIMILAR RESULTS CAN BE ACHIEVED WITH OTHER
% REDUNDANT ROBOTS

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
fprintf('\nSimple test: try to reach T with the Sawyer robot')
robot = load_robot('practicals', 'SAWYER');
adjust_view(robot)

T1 = [1   0  0  0.5; 
     0   1  0  0.5;
     0   0  1  0.5;
     0   0  0   1];
 
T2 =[0.0000    0.0000    1.0000    1.0148;
   -0.0000   -1.0000    0.0000    0.1603;
    1.0000   -0.0000   -0.0000    0.3170;
         0         0         0    1.0000];
     
T3 =[0.3164    0.0369    0.9479    0.9796;
   -0.2628   -0.9567    0.1250    0.2655;
    0.9115   -0.2886   -0.2931    0.1686;
         0         0         0    1.0000];

T = T1;
     
% try this initial seed for the inverse kinematicdifferent seeds 
% do allow to obtain different solutions
q = [0.2 -0.2 -0.4 -0.4 -0.2 -0.2 -0.2]';
% q=[0.0 0.0 0.0 0.0 0.0 0.0 0.0]';

drawrobot3d(robot, q)
qinv = inversekinematic(robot, T, q)
T_reach = directkinematic(robot, qinv)

'diff T-Treach'
T-T_reach


