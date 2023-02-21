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
function speed_demo_with_movement
close all
robot = load_robot('OMRON', 'ECOBRA600')

% DIRECT JACOBIAN
%joint position
q = [0 0 0 0]';
%joint speed
qd = [.5 .3 0.05 .2]';

step = 0.01;
t = 0:step:8;

qt = [];
qdt = [];
vwt = [];
for i=1:length(t) 
    J = manipulator_jacobian(robot, q);
    vw = J*qd;
    q = q + qd*step;
    qt = [qt q];
    qdt = [qdt qd];    
    vwt = [vwt vw];
end

figure, plot(t, qt)
title('Posición articular')
xlabel('tiempo (s)')
ylabel('q (rad)')
legend('q1 (rad)', 'q2 (rad)', 'q3 (m)', 'q4 (rad)')

figure, plot(t, qdt)
title('Velocidad articular')
xlabel('tiempo (s)')
ylabel('qd (rad)')
legend('qd1 (rad/s)', 'qd2 (rad/s)', 'qd3 (m/s)', 'qd4 (rad/s)')

figure, plot(t, vwt)
title('Velocidad en el extremo')
xlabel('tiempo (s)')
ylabel('v (m/s)')
legend('vx m/s', 'vy m/s', 'vz m/s', 'wx', 'wy', 'wz')

% animamos el resultado
%animate(robot, qt(:, 1:10:end))
% animamos el resultado con un vector de velocidad en el extremo del robot
for i=1:5:length(t)
    q = qt(:,i);
    vw = vwt(:,i);
    T = directkinematic(robot, q);
        
    %plot speed
    p0 = T(1:3,4);
    drawrobot3d(robot, q)
    draw_vector(vw(1:3), p0, '       V', 3)
    draw_vector(vw(4:6), p0, '       W', 3)
    pause(0.1)
end