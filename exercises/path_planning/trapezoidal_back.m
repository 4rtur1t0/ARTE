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
function trapezoidal
close all
%trapezoidal_single_joint()
trapezoidal_multi_joint();

function trapezoidal_single_joint()
% system requirements
qdmax = 1;
qddmax = 3;
delta_time = 0.05;

% initial and end values
q0=0.1;
qf=-2.1;
%actual speeds and accelerations
qdmax = qdmax*sign(qf-q0);
qddmax = qddmax*sign(qf-q0);
[ttotal, tcte, tacc] = compute_times(q0, qf, qdmax, qddmax);
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, qdmax, qddmax, tcte, tacc, delta_time);

figure, grid on, hold on
plot(time, qt, 'o')
figure, grid on
plot(time, qdt, 'o')
figure, grid on
plot(time, qddt, 'o')


function trapezoidal_multi_joint()
close all
f1=figure(), grid on, hold on, ylabel('q(t) (rad)'), xlabel('tiempo (s)')
f2=figure(), grid on, hold on, ylabel('$\dot{q}(t) (rad/s)$'), xlabel('tiempo (s)')
f3=figure(), grid on, hold on, ylabel('$\ddot{q}(t) (rad/s^2)$'), xlabel('tiempo (s)')

% system requirements
qdmax = [1, 10, 10];
qddmax = [2, 2, 2];
delta_time = 0.05;
% initial and end values
q0=[0.0, 0.0, 0.0];
qf=[1.0, 2.0, 3.0];
qdmax = qdmax.*sign(qf-q0);
qddmax = qddmax.*sign(qf-q0);
%store current planning times
planning_times = [];
for i=1:length(q0)
    [ttotal, tcte, tacc] = compute_times(q0(i), qf(i), qdmax(i), qddmax(i));
    planning_times =[planning_times [ttotal tcte tacc]'];
end
% find max time,
ttotalmax = max(planning_times(1, :));

% recompute qdmax* given the new time, compute
new_qdmax = [];
for i = 1:length(q0)
    [qdmaxi] = compute_new_speeds(q0(i), qf(i), ttotalmax, qdmax(i), qddmax(i));
    new_qdmax = [new_qdmax qdmaxi];
end

% demo, maintain different times
for i=1:length(q0)
    %consider the case of a cero movement.
    if abs(qf(i)-q0(i))==0
        time = 0:delta_time:ttotalmax;
        qt = qf(i)*ones(length(time));
        qdt = 0*ones(length(time));
        qddt = 0*ones(length(time));
    else
        [ttotal, tcte, tacc] = compute_times(q0(i), qf(i), new_qdmax(i), qddmax(i));
        [qt, qdt, qddt, time] = trapezoidal_profile(q0(i), qf(i), new_qdmax(i), qddmax(i), tcte, tacc, delta_time);
    end
    figure(f1)
    plot(time, qt, 'o')
    figure(f2)
    plot(time, qdt, 'o')
    figure(f3)
    plot(time, qddt, 'o')
end
figure(f1)
legend('q_1(t)','q_2(t)','q_3(t)')
figure(f2)
legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
figure(f3)
legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')


function [qt, qdt, qddt, time]=trapezoidal_profile(q0, qf, qdmax, qddmax, tcte, tacc, delta_time)
%deltaq = qf-q0;
%s = sign(deltaq);
%qdmax = s*qdmax;
%qddmax = s*qddmax;
qd0=0;
if tcte <= 0
    fprintf('Triangular profile')   
    % recompute the max speed of the profile
    qdmax = qddmax*tacc;
    q1 = q0+qddmax*tacc^2/2;
    q2 = q1+tcte*qdmax;
    % build the global time vector
    time = 0:delta_time:(tacc+tacc);
    timeA=time(time >=0 & time <=tacc); 
    timeB=[];
    timeC=time(time >(tacc+tcte))-tacc;
    [qtA, qdtA, qddtA]=acceleration([q0, qd0, qddmax], timeA); 
    qtB=[];
    qdtB=[];
    qddtB=[];
    [qtC, qdtC, qddtC]=acceleration([q2, qdmax, -qddmax], timeC);
else
    fprintf('Trapezoidal profile')
    q1 = q0+qddmax*tacc^2/2;
    q2 = q1+tcte*qdmax;     
    % generate a sampled time. Then filter each local time and
    % refer it to zero.
    time = 0:delta_time:(tacc+tacc+tcte);
    timeA=time(time >=0 & time <=tacc);
    timeB=time(time >tacc & time <=(tacc+tcte))-tacc;
    timeC=time(time >(tacc+tcte))-tacc-tcte;
    % compute profiles
    [qtA, qdtA, qddtA]=acceleration([q0, qd0, qddmax], timeA);
    [qtB, qdtB, qddtB]=constant_speed([q1, qdmax], timeB);
    [qtC, qdtC, qddtC]=acceleration([q2, qdmax, -qddmax], timeC);                
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
% build the global q(t), qd(t) and qdd(t)
qt = [qtA, qtB, qtC];
qdt = [qdtA, qdtB, qdtC];
qddt = [qddtA, qddtB, qddtC];



function [ttotal, tcte, tacc]=compute_times(q0, qf, qdmax, qddmax)
deltaq = qf-q0;
% standard trapezoidal case
tacc=qdmax/qddmax;
tcte= deltaq/qdmax - qdmax/qddmax;
% triangular case
if tcte<=0
    tcte = 0;
    tacc = sqrt(deltaq/qddmax);    
end
ttotal = tcte + 2*tacc;


%
% Given delta q and qddmax, computes two possible solutions for qdmax
% The one within specs is chosen.
%
function [qdmax]=compute_new_speeds(q0, qf, ttotal, qdmax, qddmax)

deltaq = qf - q0;

a = 1;
b= -ttotal*qddmax;
c= deltaq*qddmax;

qdmax1 = (-b+sqrt((b.^2)-(4*a*c)))/(2*a);
qdmax2 = (-b-sqrt((b.^2)-(4*a*c)))/(2*a);
if abs(qdmax1) <=abs(qdmax)
    qdmax=qdmax1;
elseif abs(qdmax2) <=abs(qdmax)
    qdmax = qdmax2;
end






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








