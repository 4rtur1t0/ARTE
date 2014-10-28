%   DIRECTKINEMATIC		Direct Kinematic for serial robots.
%
% 	T = DIRECTKINEMATIC(robot, Q) returns the transformation matrix T
%   of the end effector according to the vector q of joint coordinates.
%
%	See also DH
%
%   Author: Arturo Gil. Universidad Miguel Hernandez de Elche. 
%   email: arturo.gil@umh.es date:   01/04/2012

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
function T = directkinematic(robot, q)

%In the case of a parallel robot, switch to its particular direct kinematic
%function
if isfield(robot, 'parallel')
    %Call specific direct kinematic function for parallel robots
    T = eval(robot.directkinematic_fn);
    return;
end

%compute direct kinematics for serial robotics
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

n=length(theta); %number of DOFs

if robot.debug
    fprintf('\nComputing direct kinematics for the %s robot with %d DOFs\n',robot.name, n);
end
%load the position/orientation of the robot's base
T = robot.T0;

for i=1:n,
    T=T*dh(theta(i), d(i), a(i), alfa(i));    
end

%if there is a tool attached to it, consider it in the computation of 
% direct kinematics
% if isfield(robot, 'tool')
%     T=T*robot.tool.TCP; %dh(theta(i), d(i), a(i), alfa(i)); 
% end