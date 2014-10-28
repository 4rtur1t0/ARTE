%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   simulation_grab_piece
%   Computes the variable robot.tool.Trel that defines the relative position
%   and orientation of the piece with respect to the tool at the moment
%   when the piece is grabbed.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function simulation_release_piece 

global configuration robot

%T07=directkinematic(robot, robot.q)*robot.tool.TCP;
%robot.tool.Trel=inv(T07)*robot.piece.T0;
T07=directkinematic(robot, robot.q);
if isfield(robot, 'tool')
    T07=T07*robot.tool.TCP;
end

% the piece is now released
robot.tool.piece_gripped=0;

%save the last known pose
robot.piece.T0=T07*(robot.tool.Trel);










