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








