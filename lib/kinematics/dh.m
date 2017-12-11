%   DENAVIT Compute an homogeneous transform matrix DH in terms of
%   Denavit-Hartenberg's parameters of the robot
%
%   A = DH(TETA, D, A, alpha) returns a 4 x 4 homogeneous
%   transformation matrix  as a function of the the Denavit-Hartenberg's
%   parameters D, alpha, A and THETA for link i.
%
%   A = DH(robot, q, i) is an abbreviated call to return a 4x4
%   homogeneous transformation matrix as a function of the robot parameters
%   robot.DH.theta, d, a, alpha the joint values q and the transformation i.
%   For i = 1, the function returns the transformation matrix T01, for i=2,
%   T12 for i=6 the transformation T56..., etc.
%
%	See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche.
%   email: arturo.gil@umh.es date:   01/01/2012

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
function A=dh(theta, d, a, alpha)
switch nargin
    case 3 %abbreviated call to denavit
        %arrange arguments
        robot = theta;
        q = d;
        i = a;
        
        theta = eval(robot.DH.theta);
        d = eval(robot.DH.d);
        a = eval(robot.DH.a);
        alpha = eval(robot.DH.alpha);
        
        theta = theta(i);
        d = d(i);
        a = a(i);
        alpha = alpha(i);
        
        A=[cos(theta)  -cos(alpha)*sin(theta)   sin(alpha)*sin(theta)   a*cos(theta);
            sin(theta)   cos(alpha)*cos(theta)  -sin(alpha)*cos(theta)   a*sin(theta);
            0              sin(alpha)             cos(alpha)             d;
            0                     0                     0              1];
    
	case 4 %full 4 argumen call to denavit
        A=[cos(theta)  -cos(alpha)*sin(theta)   sin(alpha)*sin(theta)   a*cos(theta);
            sin(theta)   cos(alpha)*cos(theta)  -sin(alpha)*cos(theta)   a*sin(theta);
            0              sin(alpha)             cos(alpha)             d;
            0         0                     0              1];
    otherwise
        disp('ERROR:denavit: uncorrect number of arguments')
end
