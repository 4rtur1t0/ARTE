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
function trapezoidal_multi_joint
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on
% initial and end values
q0=[0.5, 0.5, 0.5];
qf=[1.2, 0.75, 0.8];
% system requirements in absolute value
omega_max = [0.5, 1, 2];
alpha_max = [0.5, 0.75, 1];
delta_time = 0.05;
ttotal = [];
qt = [];
qdt = [];
qddt = [];
% Plan a path using omega, alpha
% returns the total time needed for each joint
for i=1:length(q0)
    [ttotal_i, traj_type, qt_, qdt_, qddt_, time]=caseA(q0(i), qf(i), omega_max(i), alpha_max(i), delta_time);
    ttotal = [ttotal ttotal_i];
    figure(h1)
    plot(time, qt_, 'o')
    figure(h2)
    plot(time, qdt_, 'o')
    figure(h3)
    plot(time, qddt_, 'o')
end
figure(h1)
legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')
figure(h2)
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\dot{q}(t)$ (rad/s)')
figure(h3)
legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\ddot{q}(t)$ (rad/s^2)')

% find the max time 
t_coord = max(ttotal);
close all
h1=figure; grid on, hold on
h2=figure; grid on, hold on
h3=figure; grid on, hold on
%now, plan all the joints using the time for a isochronous movement.
for i=1:length(q0)
    [qt_, qdt_, qddt_, time]=caseB(q0(i), qf(i), t_coord, alpha_max(i), delta_time);
    figure(h1)
    plot(time, qt_, 'o')
    figure(h2)
    plot(time, qdt_, 'o')
    figure(h3)
    plot(time, qddt_, 'o')
    qt = [qt; qt_];
    qdt = [qdt; qdt_];
    qddt = [qddt; qddt_];
end
%close all
figure(h1)
legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')
figure(h2)
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\dot{q}(t)$ (rad/s)')
figure(h3)
legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$\ddot{q}(t)$ $(rad/s^2)$')

%
% Representamos el task space y 
% las velocidades
robot = load_robot('example', '3dofplanar');
pt = [];
phit = [];
vwt = []
for i=1:length(qt)
    q = qt(:, i);
    qd = qdt(:, i);
    T = directkinematic(robot, q);
    pt = [pt T(1:3, 4)];
    phit = [phit atan2(T(2,1), T(1,1))];
    J = manipulator_jacobian(robot, q);
    vw = J*qd;
    vwt = [vwt [vw(1:2); vw(6)]];
end
close all
h1=figure;
h2=figure;
h3=figure;
figure(h1)
plot(time, qt(1, :), 'ro'), hold on, grid on
plot(time, qt(2, :), 'go')
plot(time, qt(3, :), 'bo')
legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')

figure(h2)
plot(pt(1, :), pt(2, :), 'ro'), grid on
xlabel('x (m)')
ylabel('y (m)')

figure(h3)
plot(time, pt(1, :), 'ro'), hold on, grid on
plot(time, pt(2, :), 'go')
plot(time, phit, 'bo')
legend('$x(t)$','$y(t)$','$\phi (t)$')
xlabel('tiempo (s)')
ylabel('$x(t)$ (m), $y(t)$ (m), $\phi (t)$ (rad)')
animate(robot, qt)


% now the joint speeds
close all
h1=figure;
h2=figure;
figure(h1)
plot(time, qdt(1, :), 'ro'), hold on, grid on
plot(time, qdt(2, :), 'go')
plot(time, qdt(3, :), 'bo')
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
xlabel('tiempo (s)')
ylabel('$q(t)$ (rad)')

figure(h2)
plot(time, vwt(1, :), 'ro'), hold on, grid on
plot(time, vwt(2, :), 'go')
plot(time, vwt(3, :), 'bo')
legend('$\dot{x}(t)$','$\dot{y}(t)$','$\dot{\phi} (t)$')
xlabel('tiempo (s)')
ylabel('$\dot{x}(t)$ (m), $\dot{y}(t)$ (m), $\dot{\phi} (t)$ (rad)')
animate(robot, qt)




function [ttotal, traj_type, qt, qdt, qddt, time ]=caseA(q0, qf, omega, alpha, delta_time)
%actual speeds and accelerations with sign
omega = omega*sign(qf-q0);
alpha = alpha*sign(qf-q0);
[ttotal, traj_type] = compute_time(q0, qf, omega, alpha);
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time);

fprintf('\n\nCASE A. ttotal: %f, traj_type: %s\n', ttotal, traj_type)

% figure, grid on, hold on
% plot(time, qt, 'o')
% figure, grid on
% plot(time, qdt, 'o')
% figure, grid on
% plot(time, qddt, 'o')


function [qt, qdt, qddt, time]=caseB(q0, qf, ttotal, alpha, delta_time)
%actual acceleration with sign
alpha = alpha*sign(qf-q0);
[omega, traj_type] = compute_speed(q0, qf, ttotal, alpha);
%caution, indicating a new speed omega
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time);

fprintf('\n\nCASE B. ttotal: %f, traj_type: %s\n', ttotal, traj_type)

% figure, grid on, hold on
% plot(time, qt, 'o')
% figure, grid on
% plot(time, qdt, 'o')
% figure, grid on
% plot(time, qddt, 'o')


function [ttotal, traj_type]=compute_time(q0, qf, omega, alpha)
deltaq = qf-q0;
% standard trapezoidal case
traj_type='trapezoidal';
tacc=omega/alpha;
tcte= deltaq/omega - omega/alpha;
% triangular case
if tcte<=0
    tcte = 0;
    tacc = sqrt(deltaq/alpha);
    traj_type='triangular';
end
ttotal = tcte + 2*tacc;


%
% Given delta q and qddmax, computes two possible solutions for qdmax
% The one within specs is chosen.
%
function [omega, traj_type]=compute_speed(q0, qf, ttotal, alpha)
deltaq = qf - q0;
%compute the total time
a = 1;
b = -ttotal*alpha;
c = deltaq*alpha;
omega1 = (-b+sqrt((b.^2)-(4*a*c)))/(2*a);
omega2 = (-b-sqrt((b.^2)-(4*a*c)))/(2*a);

% this case is unfeasible. Given alpha
if imag(omega1) > 0
    disp('Unfeasible')
    disp('The movement cannot be performed with this alpha')
    disp('Increase acceleration alpha')
    omega=0;
    traj_type='unfeasible';
    return 
end

if omega1==omega2
    omega=omega1;
    traj_type='triangular';
else
    omegas =[omega1, omega2];
    [m, i] = min(abs(omegas));    
    omega=omegas(i);
    traj_type = 'trapezoidal';
end



function [qt, qdt, qddt, time]=trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time)
qd0=0;
if strcmp(traj_type, 'triangular')
    tcte=0;
    tacc = ttotal/2;
    deltaq=(qf-q0);
    %alpha=(qf-q0)/tacc^2;
    omega = deltaq/tacc;
    fprintf('Triangular profile')   
    % recompute the max speed of the profile    
    q1 = q0 + alpha*tacc^2/2;
    q2 = q1 + tcte*omega;
    % build the global time vector
    time = 0:delta_time:(tacc+tacc);
    timeA=time(time >=0 & time <=tacc); 
    timeB=[];
    timeC=time(time >(tacc+tcte))-tacc;
    [qtA, qdtA, qddtA]=acceleration([q0, qd0, alpha], timeA); 
    qtB=[];
    qdtB=[];
    qddtB=[];
    [qtC, qdtC, qddtC]=acceleration([q2, omega, -alpha], timeC);
else
    fprintf('Trapezoidal profile')
    tacc = omega/alpha;
    tcte = ttotal - 2*tacc;
    q1 = q0+alpha*tacc^2/2;
    q2 = q1+tcte*omega;     
    % generate a sampled time. Then filter each local time and
    % refer it to zero.
    time = 0:delta_time:(tacc+tacc+tcte);
    timeA=time(time >=0 & time <=tacc);
    timeB=time(time >tacc & time <=(tacc+tcte))-tacc;
    timeC=time(time >(tacc+tcte))-tacc-tcte;
    % compute profiles
    [qtA, qdtA, qddtA]=acceleration([q0, qd0, alpha], timeA);
    [qtB, qdtB, qddtB]=constant_speed([q1, omega], timeB);
    [qtC, qdtC, qddtC]=acceleration([q2, omega, -alpha], timeC);                
end

% figure, 
% plot(timeA, qtA, 'ro'), hold on, grid
% plot(timeB+tacc, qtB, 'go')
% plot(timeC+tacc+tcte, qtC, 'bo')
%     
% figure, 
% plot(timeA, qdtA, 'ro'), hold on, grid
% plot(timeB+tacc, qdtB, 'go')
% plot(timeC+tacc+tcte, qdtC, 'bo')
%     
% figure, 
% plot(timeA, qddtA, 'ro'), hold on, grid
% plot(timeB+tacc, qddtB, 'go')
% plot(timeC+tacc+tcte, qddtC, 'bo')

%build the global q(t), qd(t) and qdd(t)
qt = [qtA, qtB, qtC];
qdt = [qdtA, qdtB, qdtC];
qddt = [qddtA, qddtB, qddtC];



function [q_t, qd_t, qdd_t]=acceleration(b, time)
%second order
q_t=b(1) + b(2)*time + b(3)*time.^2/2;
% speed
qd_t= b(2) + b(3)*time;
%return a constant accel
qdd_t=b(3)*ones(1,length(time));

function [q_t, qd_t, qdd_t]=constant_speed(b, time)
%the first order equation
q_t=b(1) + b(2)*time;
% speed
qd_t= b(2)*ones(1,length(time));
%return a constant accel
qdd_t=0*ones(1,length(time));








