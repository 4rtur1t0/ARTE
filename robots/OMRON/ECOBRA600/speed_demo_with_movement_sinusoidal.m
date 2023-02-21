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
function speed_demo_with_movement_sinusoidal
close all

robot = load_robot('OMRON', 'ECOBRA600')


% example trajectories (10 seconds)
delta_t = 0.01; % s
t = 0:delta_t:20;
w1 = 0.5; %rad/s
w2 = 1.0; %rad/s
w3 = 1.5; %m/s

qt = [sin(w1*t); cos(w2*t);  0.1*(sin(w3*t)+0.3*cos(w3*t)); 0*t];

qds = [];
vs = [];

for i=1:length(t)-1
    % joint position
    q = qt(:, i);   
    % joint speed
    qd = (qt(:, i+1) - qt(:, i))/delta_t;
 
    J = manipulator_jacobian(robot, q);  
    % end effector's velocity
    v = J*qd;   
    qds = [qds qd];
    vs = [vs v];
end
qds = [qds qd];
vs = [vs v];

figure, plot(t, qt)
title('Posición articular')
xlabel('tiempo (s)')
ylabel('q (rad)')
legend('q1 (rad)', 'q2 (rad)', 'q3 (m)')

figure, plot(t, qds)
title('Velocidad articular')
xlabel('tiempo (s)')
ylabel('qd (rad)')
legend('qd1 (rad/s)', 'qd2 (rad/s)', 'qd3 (m/s)')

figure, plot(t, vs)
title('Velocidad en el extremo')
xlabel('tiempo (s)')
ylabel('v (m/s)')
legend('vx m/s', 'vy m/s', 'vz m/s')

% animamos el resultado
%animate(robot, qt(:, 1:10:end))
% animamos el resultado con un vector de velocidad en el extremo del robot
for i=1:5:length(t)
    q = qt(:,i);
    v = vs(:,i);
    T = directkinematic(robot, q);
        
    %plot speed
    p0 = T(1:3,4);
    drawrobot3d(robot, q)
    draw_vector(v(1:3), p0, '       V', 3)
    pause(0.1)
end

