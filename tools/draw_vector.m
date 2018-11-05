%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW_AXIS(V, p0, text, scale) 
%
%   Draws a vector V at the point p0. Adds
%   text
%   SCALE: change the size of the text and vectors.
%   the reference system associated is drawn with arrows X (red), Y (green), Z (blue)
%
%	See also DRAWROBOT3D, DRAW_AXES, ANIMATE
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   02/11/2018
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
function draw_vector(V, p0, text_label, scale)

if ~exist('scale', 'var')
    scale=0.8;
end
p1 = p0(:) + 0.1*scale*V;
vect_arrow(p0, p1, 'r', scale*3) %standard WIDTH for arrows is 3
text(p1(1)+0.01, p1(2)-0.01, p1(3)+0.01, text_label, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', round(24*scale)); 
