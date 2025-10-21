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
% initial and end values
q0=0.5;
qf=3.0;% Given these requirements, compute a trapezoidal profile
% system requirements in absolute value
omega_desired = 5;
alpha_desired = 5;
delta_time = 0.05;

% Plan a path using omega, alpha
% returns the total time needed. 
% returns the paths (q(t), qd(t), qdd(t))
ttotal=caseA(q0, qf, omega_desired, alpha_desired, delta_time);
close all
ttotal=3.0;
caseB(q0, qf, ttotal, alpha_desired, delta_time)



function [ttotal, traj_type]=caseA(q0, qf, omega, alpha, delta_time)
%actual speeds and accelerations with sign
omega = omega*sign(qf-q0);
alpha = alpha*sign(qf-q0);
[ttotal, traj_type] = compute_time(q0, qf, omega, alpha);
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time);

fprintf('\n\nCASE A. ttotal: %f, traj_type: %s\n', ttotal, traj_type)

figure, grid on, hold on
plot(time, qt, 'o')
figure, grid on
plot(time, qdt, 'o')
figure, grid on
plot(time, qddt, 'o')


function caseB(q0, qf, ttotal, alpha, delta_time)
%actual acceleration with sign
alpha = alpha*sign(qf-q0);
[omega, traj_type] = compute_speed(q0, qf, ttotal, alpha);
%caution, indicating a new speed omega
[qt, qdt, qddt, time] = trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, delta_time);

fprintf('\n\nCASE B. ttotal: %f, traj_type: %s\n', ttotal, traj_type)

figure, grid on, hold on
plot(time, qt, 'o')
figure, grid on
plot(time, qdt, 'o')
figure, grid on
plot(time, qddt, 'o')


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

% x=-15:.1:15;
% f=a*x.^2+b*x+c;
% plot(x, f)
% 
% tcte1 = ttotal-2*omega1/alpha;
% tcte2 = ttotal-2*omega2/alpha;

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

figure, 
plot(timeA, qtA, 'ro'), hold on, grid
plot(timeB+tacc, qtB, 'go')
plot(timeC+tacc+tcte, qtC, 'bo')
    
figure, 
plot(timeA, qdtA, 'ro'), hold on, grid
plot(timeB+tacc, qdtB, 'go')
plot(timeC+tacc+tcte, qdtC, 'bo')
    
figure, 
plot(timeA, qddtA, 'ro'), hold on, grid
plot(timeB+tacc, qddtB, 'go')
plot(timeC+tacc+tcte, qddtC, 'bo')

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








