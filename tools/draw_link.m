%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW_LINK(ROBOT, I, T) 
%
%   Draws the I link of the robot according to the transformation matrix T,
%   where:
%   ROBOT is the current object storing the robot parameters. The graphical
%   representation of the robot is stored in the variables 
%   robot.graphical.link{1}.v --> robot base
%   robot.graphical.link{2}.v --> link 1...
%
%   If the robot has graphics the variable robot.graphical.has_graphics
%   must be set.
%
%   I is the current link number being drawn.
%
%   T is the global transformation that relates the base reference system
%   of the robot with the link's reference system.
%
%   If the variable robot.graphical.draw_axes is set, the reference system
%   associated with link i is drawn with arrows X (red), Y (green), Z (blue)
%
%	See also DRAWROBOT3D, ANIMATE
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/02/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function draw_link(robot, i, T)

if robot.graphical.has_graphics
    %Now draw points
    %obtain arm points
    V=robot.graphical.link{i}.v;
    V(:,4) = ones(length(V),1); %homogeneous coordinates
    
    %transform points according to current coordinates
    V = (T*V')';
    V  = V(:,1:3);
    %set robot.graphical.color to add a desired color to your robot
    draw_patch(robot.graphical.link{i}.f,V,robot.graphical.color, robot.graphical.draw_transparent);
end