% SCRIPT TEST FOR THE UR10 ROBOT KINEMATICS

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
robot = load_robot('practicals', 'UR10');
adjust_view(robot)

% DESCOMENTE LA LINEA SIGUIENTE PARA PROBAR UN PUNTO SINGULAR
q0 = [0.1 -pi/2 pi/2 0.1 0.1 0.1]';
%q0 = [0 0 0 0 0 0]';

fprintf('\nSimple test: try to reach T')
T1 = [-0.3390   -0.3390    0.8776    0.2447;
     0.8469    0.2962    0.4416   -0.4373;
    -0.4096    0.8929    0.1867    1.2568;
         0         0         0    1.0000];

T2 =[0.9018   -0.3860    0.1942   -0.1737;
    0.1903   -0.0487   -0.9805   -0.2743;
    0.3879    0.9212    0.0295    1.4111;
         0         0         0    1.0000];

T3 =[1.0000         0         0    0.0000;
         0    0.0000   -1.0000   -0.2560;
         0    1.0000    0.0000    1.4280;
         0         0         0    1.0000];

     
T4 =[1.0000         0         0    0.0000;
         0    0.0000   -1.0000   0.0;
         0    1.0000    0.0000    1.4280;
         0         0         0    1.0000];
     
% Intente llegar a T1, T2,  T3 o T4
T = T4;

% Llame a la cinematica inversa 
qinv = inversekinematic(robot, T, q0)

T_reach = directkinematic(robot, qinv)
'diff T-Treach'
T-T_reach

% Plot manipulatiliby ellipse
drawrobot3d(robot, qinv)

