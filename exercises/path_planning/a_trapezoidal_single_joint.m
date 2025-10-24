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
function trapezoidal_single_joint
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on
% initial and end values
q0 = [0.5];
qf = [1.2];
% system requirements in absolute value
omega_max = [0.5];
alpha_max = [0.5];
delta_time = 0.05;

[qt, qdt, qddt, time]= trapezoidal_coordinated(q0, qf, omega_max, alpha_max, delta_time);

figure(h1)
plot(time, qt)
legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')

figure(h2)
plot(time, qdt)
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\dot{q}(t)$ (rad/s)')

figure(h3)
plot(time, qddt)
legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\ddot{q}(t)$ (rad/s^2)')








