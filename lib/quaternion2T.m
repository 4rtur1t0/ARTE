%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   T = quaternion2T(Q, P)
%   Returns an homogeneous transformation matrix corresponding to the orientation
%   defined by quaternion Q and position defined by P
%   
%   See also QPROD, T2QUATERNION.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. email:
%   arturo.gil@umh.es date:   21/04/2012
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
function T = quaternion2T(Q, P)
    
if nargin==1
    Q = Q/norm(Q); % Ensure Q has unit norm
    
    %Set up convenience variables
    w = Q(1); x = Q(2); y = Q(3); z = Q(4);
    w2 = w^2; x2 = x^2; y2 = y^2; z2 = z^2;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;
    
    T = [w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,  0
         2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,  0
         2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2 ,  0
              0      ,       0     ,       0     ,  1];
          
          
else
    
    Q = Q/norm(Q); % Ensure Q has unit norm
    
    w = Q(1); x = Q(2); y = Q(3); z = Q(4);
    w2 = w^2; x2 = x^2; y2 = y^2; z2 = z^2;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;
    
    T = [w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,  P(1)
         2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,  P(2)
         2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2 ,  P(3)
              0      ,       0     ,       0     ,  1];
end