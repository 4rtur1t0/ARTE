% SCRIPT TEST TO CREATE A SIMULATED WORKSPACE USING A MONTE-CARLO ALGORITHM

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

function workspace_demo()
close all
fprintf('\nTHE DEMO PRESENTS A MONTE-CARLO COMPUTATION OF THE WORKSPACE')
%base position of the robot
q = [0 0 0 0 0 0];
% Number of joint posisitions to be simulated
M = 10000;
robot=load_robot('ABB', 'IRB140'); 
adjust_view(robot)
drawrobot3d(robot, q)

%Now compute direct kinematics for this position q
pp = [];
for i=1:M
    %q = [2*pi*rand-pi 2*pi*rand-pi 2*pi*rand-pi 2*pi*rand-pi 2*pi*rand-pi 2*pi*rand-pi];
    q = rand_q(robot);
    T = directkinematic(robot, q);
    %drawrobot3d(robot, q)
    %pause(0.5)
    p = T(1:3,4);
    pp = [pp p];
end
plot3(pp(1,:),pp(2,:),pp(3,:),'r.')



function q= rand_q(robot)

q=[];
for i=1:robot.DOF
    q(i) = (robot.maxangle(i,2)-robot.maxangle(i,1))*rand + robot.maxangle(i,1);
end