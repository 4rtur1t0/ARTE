%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q=QPROD(Q1, Q2)
%   Computes the product of two quaternionts Q1 and Q2
%
%	See also QUATERNIONT2T, T2QUATERNION.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. email:
%   arturo.gil@umh.es date:   02/04/2012
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
function q = qprod(q1, q2)

q = [q1(1)*q2(1)-q1(2:4)*q2(2:4)', q1(1)*q2(2:4)+q2(1)*q1(2:4)+cross(q1(2:4),q2(2:4))];