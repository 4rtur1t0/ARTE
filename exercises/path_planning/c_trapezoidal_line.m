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
function trapezoidal_line
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on

% initial and end target points (XYZ phi)
tp0 = [-1.0, 1.5, 0.0, 3*pi/4]; 
tpf = [1.2, 2.0, 0.0, pi/4];
dxyz = tpf(1:3) - tp0(1:3);
dabg = tpf(4) - tp0(4);

% planning in distances
s0 = [0 0];
sf = [norm(dxyz) norm(dabg)];

% system requirements in absolute value
v_max = [0.5 0.25]; %(m/s, rad/s)
a_max = [0.25 0.125]; %(m/s/s, rad/s/s)
delta_time = 0.05;

% plan using the trapezoidal coordinated in the given space
[st, sdt, sddt, time]= trapezoidal_coordinated(s0, sf, v_max, a_max, delta_time);
plot_distances(st, sdt, sddt, time)

% now propagate the planning to the needed XYZ coordinates
u = dxyz/norm(dxyz);
pt = u'.*st(1, :) + tp0(1:3)';
pdt = u'.*sdt(1, :);
pddt = u'.*sddt(1, :);
% alternativamente, se pued interpolar entre 
% diferentes angulos de Euler
v = dabg/norm(dabg);
phit = v*st(2, :)+ tp0(4);
phidt = v*sdt(2, :);
phiddt = v*sddt(2, :);

plot_task_space(pt, pdt, pddt, phit, phidt, phiddt, time);

% plan
robot = load_robot('example', '3dofplanar');
[qt, qdt] = plan_joint_space(robot, pt, pdt, phit, phidt);
plot_joint_space(qt, qdt, time);
animate(robot, qt)

function [qt, qdt] = plan_joint_space(robot, pt, pdt, phit, phidt)
% Now compute a path on joint coordinate space
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on

% Calculamos las posiciones y velocidades
% articulares para una planificación preliminar
qt = [];
qdt = [];
% Selection from q inverse
% a) select the initial solution with q(2)>=0 for i==1 (first iteration)
% b) select the closest solution
for i=1:length(pt)
    p = pt(1:3, i);
    phi = phit(i);
    % Speed linear/angular;
    vw = [pdt(1:2, i); phidt(i)];
    T = [cos(phi) -sin(phi) 0 p(1);
         sin(phi) cos(phi)  0 p(2);
            0         0     1  p(3) ;
            0         0     0   1];
    % inversekinematic in position
    qi = inversekinematic(robot, T); 
    % select the initial solution with q(2) >=0 (elbow down)
    if i==1
        idx = find(qi(2,:)>=0);
        q = qi(:, idx(1));
    else
        deltaq = qi-q;
        d = [norm(deltaq(:,1)) norm(deltaq(:,2))];
        [k, idx] = min(d);
        q = qi(:, idx);
    end
    % inversekinematic in speed
    J = manipulator_jacobian(robot, q);
    % make it square
    J = [J(1:2, :); J(6,:)];
    qd = inv(J)*vw;    
    qt = [qt q];
    qdt = [qdt qd];
end


function plot_distances(st, sdt, sddt, time)
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on

figure(h1)
plot(time, st)
legend('$s(t)_{xyz}$','$s(t)_{\phi}$')
xlabel('tiempo (s)')
ylabel('$s(t)$ (m, rad)')
% filename='/home/arvc/Escritorio/temp/trapezoidal_st_phit.png';
% styleplot(gca, 2, 16, 'Times New Roman', filename)

figure(h2)
plot(time, sdt)
legend('$\dot{s}(t)_{xyz}$','$\dot{s}(t)_{\phi}$')
xlabel('tiempo (s)')
ylabel('$\dot{s}(t)$ (m/s, rad/s)')

figure(h3)
plot(time, sddt)
legend('$\ddot{s}(t)_{xyz}$','$\ddot{s}(t)_{\phi}$')
xlabel('tiempo (s)')
ylabel('$\ddot{s}(t)$ ($m/s^2$, $rad/s^2$)')

function plot_task_space(pt, pdt, pddt, phit, phidt, phiddt, time)
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on
h4=figure; grid on, hold on

figure(h1)
plot(time, pt)
plot(time, phit)
legend('$x(t)$','$y(t)$', '$z(t)$', '$\phi(t)$')
xlabel('tiempo (s)')
ylabel('$x(t), y(t), \phi(t)$ (m, rad)')

figure(h2)
plot(time, pdt)
plot(time, phidt)
legend('$\dot{x}(t)','$\dot{y}$', '$\dot{z}$' ,'$\dot{\phi}$')
xlabel('tiempo (s)')
ylabel('$\dot{x}(t), \dot{y}(t), \dot{\phi}(t)$ (m, rad)')

figure(h3)
plot(time, pddt)
plot(time, phiddt)
legend('$\ddot{x}(t)','$\ddot{y}$', '$\ddot{z}$' ,'$\ddot{\phi}$')
xlabel('tiempo (s)')
ylabel('$\ddot{x}(t), \ddot{y}(t), \ddot{\phi}(t)$ (m, rad)')

figure(h4)
plot(pt(1,:), pt(2,:))
xlabel('X (m)')
ylabel('Y (m)')


function plot_joint_space(qt, qdt, time)
close all
h1=figure;
h2=figure;
%h3=figure;
figure(h1)
plot(time, qt), hold on, grid on
legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')

figure(h2)
plot(time, qdt), hold on, grid on
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\dot{q}(t)$ (rad)')



