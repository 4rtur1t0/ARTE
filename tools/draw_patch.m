%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW_PATCH(F, V, C, transparent)
%
%   Draws a link using faces F and vertices V.
%   C Defines the color of the robot.
%   If transparent == 1, the robot is drawn with transparency.
%   
%   Example of use. Move to the directory arte/robots/ABB/IRB140, for example:
%   >> [f, v, c] = stl_read('link0.stl')
%   >> draw_patch(f, v)
%   
%	See also DRAW_LINK, DRAWROBOT3D, ANIMATE
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. 
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
function draw_patch(F, V, C, transparent)
global configuration

if nargin==2
    %default color
    C=[0.5 0.6 0.7];
    transparent=0;
end
if nargin==3
    transparent=0;
end
%Following github/banenky recommendation we have removed this line
%since it is no longer supported by Matlab 2015
%set(gca, 'drawmode', 'fast');

%draw the vertices
%note: vertices should be expressed in m
p = patch('faces', F, 'vertices', V);


set(p, 'facec', 'flat');          
%set(p, 'FaceVertexCData', C);       % Set the color (from file)
set(p, 'FaceColor', C);

if transparent
    set(p, 'facealpha',.4)          % Draws the link with transparency
end

set(p, 'EdgeColor','none');         

%avoid creating more than 8 ligths. Delete any previous lights
delete(findall(gcf,'Type','light'))
light;

%change material properties
material( [0.5 0.5 0.01]);
daspect([1 1 1])                    % Setting the aspect ratio

xlabel('X (m)'),ylabel('Y (m)'),zlabel('Z (m)')
