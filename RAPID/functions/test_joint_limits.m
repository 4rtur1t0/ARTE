%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [joint, time] = test_joint_limits(robot)
%
%   Test whether any of the joint angles exceeds the mechanical
%   limits imposed by manufacturer.
%
%   The planned movement in joint coordinates must be stored in the 
%   variable robot.q_vector, where each row stores the trajectory
%   corresponding to each of the joint coordinates.
%   The function returns the joint that caused the fault and the
%   time when it occurred.
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
function [joint, time] = test_joint_limits(robot)


%if exist('robot.maxangle', 'var')    
if ~isempty(robot.maxangle)
    for i=1:robot.DOF,
        [index1]=find(robot.q_vector(i,:) < robot.maxangle(i,1));
        [index2]=find(robot.q_vector(i,:) > robot.maxangle(i,2));
        
        if ~isempty(index1)
            fprintf('\nERROR: joint out of range');
            fprintf('\nJoint %d exceeds %f (rad)', i, robot.maxangle(i,2));
        end
        if ~isempty(index2)
            fprintf('\nERROR: joint out of range');
            fprintf('\nJoint %d below %f (rad)', i, robot.maxangle(i,1));
        end
    end    
else
    disp('WARNING:test_joint_limits: robot.maxangle undefined');
end

