%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  poly = spline(tacel, thetaini, thetafinal, velini, velfinal, acelini)
%
%  Computes the coefficients a b c d e of the polynomial defined as
%  theta(t) = a + bt + ct^2 + dt^3 + et^4
%  Returns: poly = [a b c d e]
%  Inputs:
%   tacel: total time to perform movement
%   thetaini, thetafinal: initial and final joint coordinates.
%   velini, velfinal: initial and final joint speeds.
%   acelini: initial acceleration of the joints.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   26/06/2012
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
function [theta, thetap, thetapp, poly] = single_joint_spline(thetaini, thetafinal, velini, velfinal, time_vector)

tfinal = time_vector(end);

A=[1 0 0 0;
   1 tfinal tfinal^2 tfinal^3;
   0 1 0 0;
   0 1 2*tfinal 3*tfinal^2];

b = [thetaini thetafinal velini velfinal]';

x = inv(A)*b;

poly = x;

t=time_vector(:); %0:0.001:tacel;

theta = x(1) + x(2)*t + x(3)*t.^2+ x(4)*t.^3;

thetap = x(2)+2*x(3)*t+3*x(4)*t.^2;

thetapp = 2*x(3)+6*x(4)*t;

%close all
% figure, plot(t, theta)
% figure, plot(t, thetap)
% figure, plot(t, thetapp)
