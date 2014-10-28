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

robot =  load_robot('mitsubishi','pa-10');


%NOA matrix initial point
T1=[1 0 0 0.5;
    0 1 0 -0.3;
    0 0 1 0.7; 
    0 0 0  1]
%NOA matrix end point
T2=[1 0 0 0.5;
    0 1 0 0.4;
    0 0 1 0.7; 
    0 0 0  1]

%distancia entre puntos consecutivos
delta = 0.02;

punto_inicial = T1(1:3,4);
punto_final = T2(1:3,4);

v=(punto_final-punto_inicial);
v=delta*v/norm(v); %vector normalizado en la dirección de la recta
distancia = sqrt((punto_final-punto_inicial)'*(punto_final-punto_inicial));
%Generación de puntos en la trayectoria
num_points = floor(distancia/delta);
puntos = punto_inicial;
for i=1:num_points,
    puntos=[puntos i*v+punto_inicial];
end
puntos=[puntos punto_final];

figure, hold on, grid, plot3(puntos(1,:),puntos(2,:),puntos(3,:)), title('Trajectory in space'), xlabel('X (m)'), ylabel('Y (m)')

qs=[];
for i=1:length(puntos),
    T1(1:3,4)=puntos(1:3,i);    
    qinv = inversekinematic(robot, T1);
    
    %select the joint coordinates in qinv which are closest to the 
    %current joint position robot.q
    q=select_closest_joint_coordinates(qinv, robot.q);
    qs=[qs q];
    robot.q=q;%update robot.q here
end

drawrobot3d(robot, qs(:,1))
adjust_view(robot)
drawrobot3d(robot, qs(:,end))

%Now, animate the robot in 3D
animate(robot, qs);

figure, hold, plot(qs(1,:), 'r.'),plot(qs(2,:), 'g.'), plot(qs(3,:), 'b.'), plot(qs(4,:), 'c.'), 
plot(qs(5,:), 'm.'), plot(qs(6,:), 'y.'),
legend('q_1 (rad)','q_2 (rad)','q_3 (rad)', 'q_4 (rad)', 'q_5 (rad)', 'q_6 (rad)' ), title('Joint trajectories'), xlabel('Step number')

