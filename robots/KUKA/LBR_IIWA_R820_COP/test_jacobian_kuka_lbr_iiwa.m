% SCRIPT TEST FOR THE KUKA LBR ROBOT KINEMATICS

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
robot = load_robot('KUKA', 'LBR_IIWA_R820_COP');
close all

% test both solutions: based on the transpose an on the Moore-Penrose
q = -pi/2*[1,1,1,1,1,1,1]';
drawrobot3d(robot, q)
targetposition = [0.5, -0.5, 0.5]';
vmag = 0.3;
delta_time = 50/1000;
qs = []
for i=1:20
    T = directkinematic(robot, q);
    J = manipulator_jacobian(robot, q);
    J
    error = targetposition-T(1:3, 4);
    vref = vmag*error/norm(error);
    wref = [0 0 0]';
    vwref = [vref; wref];
    iJ = pinv(J)
    qd = iJ*vwref;
    q = q + qd*delta_time;
    qs = [qs q];
    drawrobot3d(robot, q)  
    pause(0.01)
    norm(error)
    if norm(error) < 0.01
        break
    end
end

norm(error)
figure, plot(qs', 'LineWidth',3.0)
legend('q1', 'q2','q3','q4','q5','q6','q7')