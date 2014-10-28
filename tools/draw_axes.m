%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW_AXES(T, X_text, Y_text, Z_text, scale) 
%
%   Draws a reference system according to the transformation matrix T. Adds
%   text X_text, Y_text, Z_text to X, Y and Z axes
%   SCALE: change the size of the text and vectors.
%   the reference system associated is drawn with arrows X (red), Y (green), Z (blue)
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
function draw_axes(T, X_text, Y_text, Z_text, scale)

if ~exist('scale', 'var')
    scale=0.8;
end

%origin p0
p0=T(1:3,4)';
x = p0 + 0.1*scale*T(1:3,1)';
y = p0 + 0.1*scale*T(1:3,2)';
z = p0 + 0.1*scale*T(1:3,3)';

vect_arrow(p0, x, 'r', scale*3) %standard WIDTH for arrows is 3
vect_arrow(p0, y, 'g', scale*3)
vect_arrow(p0, z, 'b', scale*3)
%plot vector names X_i Y_i Z_i
%standard fontsize is 
text(x(1)+0.01, x(2)-0.01, x(3)+0.01,X_text, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', round(24*scale)); 
text(y(1)+0.01, y(2)+0.01, y(3)+0.01, Y_text, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', round(24*scale));
text(z(1)+0.01, z(2)+0.01, z(3)+0.01, Z_text, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', round(24*scale));
