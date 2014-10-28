% FOLLOW A LINE IN SPACE WITH A SCARA ROBOT

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

%Compute points in line
close all


robot =  load_robot('example','scara');
%matriz de posición/orientación
T=[1  0  0 0;
   0 -1  0 0;
   0  0 -1 0;
   0  0  0 1]
%distancia entre puntos consecutivos
delta = 0.001;

punto_inicial = [0.7 0 0];
punto_final = [0 0.8 0];

V=(punto_final-punto_inicial);
d=delta*V/norm(V); %vector normalizado en la dirección de la recta
distancia = sqrt((punto_final-punto_inicial)*(punto_final-punto_inicial)');
%Generación de puntos en la trayectoria
num_points = distancia/delta;
puntos = punto_inicial;
for i=1:num_points,
    puntos=[puntos; i*d+punto_inicial];
end
puntos=[puntos;punto_final];

figure, plot(puntos(:,1),puntos(:,2)), title('Trayectoria en el espacio'), xlabel('X (m)'), ylabel('Y (m)')

qs=[];
vqs=[];
for i=1:length(puntos),
    T(1,4)=puntos(i,1);
    T(2,4)=puntos(i,2);
    
    q = inversekinematic(robot, T);
    vq = compute_joint_velocity(robot, q(:,1), V/norm(V)');
    %vq = compute_joint_velocity(robot, q(:,2), V/norm(V)');
    qs=[qs q(:,1)];
    vqs = [vqs vq];
end

figure, hold, plot(qs(1,:), 'r'),plot(qs(2,:), 'g'), plot(qs(3,:), 'b'), plot(qs(4,:), 'c'),
legend('q_1 (rad)','q_2 (rad)','q_3 (rad)', 'q_4 (rad)'), title('Joint trajectories'), xlabel('Step number')

figure, hold, plot(vqs(1,:), 'r'),plot(vqs(2,:), 'g'), plot(vqs(3,:), 'b'),
legend('vq_1 (rad/s)','vq_2 (rad/s)','vq_3 (rad/s)'), title('Joint speeds (rad/s)')

% figure,hold, plot(vqs(1,:)*30/pi, 'r'),plot(vqs(2,:)*30/pi, 'g'), plot(vqs(3,:)*30/pi, 'b'),
% legend('vq_1 (r.p.m.)','vq_2 (r.p.m.)','vq_3 (r.p.m.)'),  title('Joint speeds (r.p.m.)')