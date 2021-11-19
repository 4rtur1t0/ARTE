% Copyright (C) 2016, by Arturo Gil Aparicio
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
function jacobian_gimbal_lock
syms th_p th_r th_y
close all
[Ry, Rz, Rx] = Rot();
Rpr = Ry*Rz;
Rsb = Ry*Rz*Rx;
ys = [0 1 0]';
zs = Ry(1:3, 3);
xs = Rpr(1:3, 1);
J = [ys zs xs];
singularities = det(J)
singularities = simplify(singularities)
solve(singularities==0)
%th_p=0.3;
th_r=pi/2;
%th_y=0;
eval(J)
Rtot=eval(Rsb);
hold on
draw_axes(eye(3), 'Xs', 'Ys', 'Zs', 0.5)
draw_axes(Rtot, 'XB', 'YB', 'ZB', 0.5)
axis equal
grid on
hold off
'debug'




function [Ry, Rz, Rx] = Rot()
syms th_p th_r th_y

Ry = [cos(th_p)  0 sin(th_p);
        0      1   0;
      -sin(th_p) 0   cos(th_p)];
  
Rz = [cos(th_r) -sin(th_r) 0;
       sin(th_r) cos(th_r) 0;
       0        0       1];
   
Rx = [1    0           0;
      0 cos(th_y) -sin(th_y);
      0  sin(th_y)   cos(th_y)];

