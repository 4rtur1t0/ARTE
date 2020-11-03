% SIMPLE ALGORITHM TO FOLLOW A LINE IN SPACE.
% Copyright (C) 2019, by Arturo Gil Aparicio
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
function speed_demo_line
close all
% velocidad lineal entre puntos consecutivos
abs_linear_speed = 0.5; % (m/s)
delta_time = 0.1;

robot = load_robot('ABB', 'IRB52')

fprintf('\nPRESS ANY KEY TO CONTINUE...')
pause

%NOA matrix initial point
T1=[1 0 0 0.8;
    0 1 0 -0.3;
    0 0 1 0.9; 
    0 0 0  1]
%NOA matrix end point
T2=[1 0 0 0.5;
    0 1 0 0.4;
    0 0 1 0.7; 
    0 0 0  1]

punto_inicial = T1(1:3,4);
punto_final = T2(1:3,4);
% vector velocidad en la direcci�n de la trayectoria
v = (punto_final-punto_inicial);
v = abs_linear_speed*v/norm(v); %vector normalizado en la direcci�n de la recta
w = [0 0 0]';
xd = [v; w];

qinv = inversekinematic(robot, T1);

%Select arbitrarily the first solution
q = qinv(:,1);

for i=1:1000
   J = manipulator_jacobian(robot, q);
   qd = inv(J)*xd;
   q = q + delta_time*qd;
   drawrobot3d(robot, q)
   pause(0.1)
end

