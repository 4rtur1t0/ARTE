% SIMPLE ALGORITHM TO FOLLOW A LINE IN SPACE. Error correction based on a P
% controller on the closest point to the line vector.
%
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
function path_planning_line_simple
close all
% velocidad lineal entre puntos consecutivos
abs_linear_speed = 0.5; % (m/s)
delta_time = 0.05;
epsilon = 0.02;
kp = 10;

robot = load_robot('ABB', 'IRB52')
q = [0 0 0 0 0 0 0];

fprintf('\nPRESS ANY KEY TO CONTINUE...')
pause

%NOA matrix initial point
T1=[1 0 0 0.8;
    0 1 0 -0.5;
    0 0 1 0.3; 
    0 0 0  1];
%NOA matrix end point
T2=[1 0 0 0.5;
    0 1 0 0.4;
    0 0 1 0.7; 
    0 0 0  1];

start_point = T1(1:3,4);
end_point = T2(1:3,4);
% vector velocidad en la direcci�n de la trayectoria
v = (end_point-start_point);
v = abs_linear_speed*v/norm(v); %vector normalizado en la direcci�n de la recta
w = [0 0 0]';
xd = [v; w];

% Solve inverse kinematics at first position
qinv = inversekinematic(robot, T1, q);

%Select arbitrarily the first solution
q = qinv(:,1);
qs = [];
qds = [];

errors_line = [];
% using a general Moore-Penrose pseudo inverse
while 1
   J = manipulator_jacobian(robot, q);
   qd1 = inv(J)*xd;
   q = q + delta_time*qd1;
   T = directkinematic(robot, q);
   p = T(1:3, 4);
   % find the error of p with respect to the line.
   [delta_end, error_line, error_line_vector] = find_errors(start_point, end_point, p);  
   qd2 = pinv(J)*[error_line_vector' 0 0 0]';
   q = q + kp*delta_time*qd2;
   if delta_end < epsilon
       fprintf('Finished trajectory')
       break
   end
   errors_line = [errors_line error_line];
   qs = [qs q];
   qds = [qds qd1];
   drawrobot3d(robot, q)
   line([start_point(1) end_point(1)], [start_point(2) end_point(2)] , [start_point(3) end_point(3)] )
end

figure, plot(errors_line), title('M�nimo error con la recta'), xlabel('Num. de movimiento'), ylabel('Error (m)')
figure, plot(qs'),  title('Coordenadas articulares'), xlabel('Movement number'), ylabel('Position (rad)')
figure, plot(qds'),  title('Velocidades articulares'), xlabel('Movement number'), ylabel('Speed (rad/s)')

% find:
% error_end: the error with respect to the end point
% error_line: the error of p to the line defined b point a and vector n.
% the line is defined by points a and b.
function [error_end, error_line, error_line_vector]=find_errors(a, b, p)
error_end = sqrt((p-b)'*(p-b));

% define the line as a, n
n = (b-a);
n = n/norm(n);

error_line = (a-p)-((a-p)'*n)*n;
error_line = norm(error_line);

% Now obtain current point minus initial point a
% project
w = p-a;
p_ = n*dot(w, n);
% add the origin since the point p_ is referred to the line
p_ = p_ + a;
% find a vector connecting the current point p and the point belonging to
% the line p_
error_line_vector = p_ - p;
