% coordinated isochronous trapezoidal planner
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
function [st, sdt, sddt, time]= trapezoidal_coordinated(s0, sf, v_max, a_max, delta_time)
ttotal = [];
st = [];
sdt = [];
sddt = [];

% h1=figure; grid on, hold on
% h2=figure; grid on, hold on
% h3=figure; grid on, hold on


% Plan a path in task space
for i=1:length(s0)
    [ttotal_i, traj_type, st_, sdt_, sddt_, time]=caseA(s0(i), sf(i), v_max(i), a_max(i), delta_time);
    ttotal = [ttotal ttotal_i];
%     figure(h1)
%     plot(time, st_, 'o')
%     figure(h2)
%     plot(time, sdt_, 'o')
%     figure(h3)
%     plot(time, sddt_, 'o')
end
% figure(h1)
% legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$q(t)$ (rad)')
% figure(h2)
% legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$\dot{q}(t)$ (rad/s)')
% figure(h3)
% legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$\ddot{q}(t)$ (rad/s^2)')

% find the max time 
t_coord = max(ttotal);
% close all
% h1=figure; grid on, hold on
% h2=figure; grid on, hold on
% h3=figure; grid on, hold on
%now, plan all the joints using the time for a isochronous movement.
for i=1:length(s0)
    [st_, sdt_, sddt_, time]=caseB(s0(i), sf(i), t_coord, a_max(i), delta_time);
%     figure(h1)
%     plot(time, st_, 'o')
%     figure(h2)
%     plot(time, sdt_, 'o')
%     figure(h3)
%     plot(time, sddt_, 'o')
    st = [st; st_];
    sdt = [sdt; sdt_];
    sddt = [sddt; sddt_];
end
%close all
% figure(h1)
% legend('$q_1(t)$','$q_2(t)$','$q_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$q(t)$ (rad)')
% figure(h2)
% legend('$\dot{q}_1(t)$','$\dot{q}_2(t)$','$\dot{q}_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$\dot{q}(t)$ (rad/s)')
% figure(h3)
% legend('$\ddot{q}_1(t)$','$\ddot{q}_2(t)$','$\ddot{q}_3(t)$')
% xlabel('tiempo (s)')
% ylabel('$\ddot{q}(t)$ $(rad/s^2)$')






function [ttotal, traj_type, qt, qdt, qddt, time ]=caseA(q0, qf, omega, alpha, delta_time)
%actual speeds and accelerations with sign
omega = omega*sign(qf-q0);
alpha = alpha*sign(qf-q0);
[ttotal, traj_type] = compute_time(q0, qf, omega, alpha);
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time);
%(q0, qdmax, qddmax, tcte, tacc, delta_time)
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
% standard trapezoidal case. This will be returned by default.
traj_type='trapezoidal';
% deal with the case in which no movement should be produced
if abs(deltaq) == 0
    ttotal = 0;
    traj_type = 'nomovement';
    return
end
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
% in this case, the deltaq=0 is handled correctly by default, returning a
% triangular trajectory that will be correctly generated
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









