%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Test whether any of the joint angles exceeds the mechanical
%  limits imposed by manufacturer
%   
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche
%   Date: 05/05/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
function error=test_joints(robot, q)


%0--> no error
error = 0;
if ~isempty(robot.maxangle)
    for i=1:robot.DOF,
        
        if q(i) < robot.maxangle(i,1)
            fprintf('\nERROR: joint out of range');
            fprintf('\nJoint %d, value %f below %f (rad)', i, q(i), robot.maxangle(i,1));
            error = 1; % an error has occurred
        end
        if q(i) > robot.maxangle(i,2)
            fprintf('\nERROR: joint out of range');
            fprintf('\nJoint %d, value %f over %f (rad)', i, q(i), robot.maxangle(i,2));
            error = 1; % an error has occurred
        end
    end
    
else
    disp('WARNING: lib/test_joints: robot.maxangle undefined');
end
