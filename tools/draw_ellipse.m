%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DRAW_ELLIPSE(POS, COV, COLOR) 
%   Draws an ellipse in the current figure with color COLOR, 'r', 'g', 'b'...etc 
%   The ellipse is drawn at position POS and defined by its covariance
%   matrix COV.
%   
%	See also DRAW_CIRCLE, DRAWROBOT3D.
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
function h = draw_ellipse(pos, cov, color)

%for different DOF 
chi2=[6.6349    9.2103   11.3449   13.2767   15.0863   16.8119   18.4753];

tita = linspace(0, 2*pi,40);
CIRCLE = [cos(tita); sin(tita)];

[V,D]=eig(full(cov(1:2,1:2)));
ejes=sqrt(chi2(2)*diag(D));
P = (V*diag(ejes))*CIRCLE;
hp = line(P(1,:)+pos(1), P(2,:)+pos(2));
set(hp,'Color', color);

