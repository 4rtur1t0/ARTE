%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VECT_ARROW(P0, P1, COLOR) 
%
% VECT_ARROW(P0, P1) plots a line vector with arrow pointing from point P0
%   to point P1. 
%
%   Example:
%       %3D vector
%       p0 = [1 2 3];   % First point p0
%       p1 = [4 5 6];   % Second point p1
%       vectarrow(p0, p1)
%
% See also DRAW_LINK, DRAW_PATCH, DRAWROBOT3D.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   25/02/2012
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
function vect_arrow(p0, p1, color, width)

if ~exist('width', 'var')
    w=3;
else
    w=width;
end

x0 = p0(1);
y0 = p0(2);
z0 = p0(3);
x1 = p1(1);
y1 = p1(2);
z1 = p1(3);
plot3([x0;x1],[y0;y1],[z0;z1], color, 'LineWidth',w);   % Draw a line between p0 and p1

p = p1-p0;
alpha = 0.1;
beta = 0.1;

hu = [x1-alpha*(p(1)+beta*(p(2)+eps)); x1; x1-alpha*(p(1)-beta*(p(2)+eps))];
hv = [y1-alpha*(p(2)-beta*(p(1)+eps)); y1; y1-alpha*(p(2)+beta*(p(1)+eps))];
hw = [z1-alpha*p(3);z1;z1-alpha*p(3)];

plot3(hu(:),hv(:),hw(:), color)  % Plot arrow head
  