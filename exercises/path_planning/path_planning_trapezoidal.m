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
function path_planning_trapezoidal
close all
q1=0;
q2=3*pi/4;
qdmax=pi/5; %rad/s
taccel= 0.2;
tini=0;
tfin=(q2-q1)/qdmax+2*taccel;
delta_t=0.01;
fifth_order([tini tini+taccel tfin-taccel tfin], [q1 q2], qdmax, delta_t)


function fifth_order(t, q, qdmax, delta_t)
%   quinto orden

A=[1 t(1) t(1)^2 t(1)^3 t(1)^4 t(1)^5;
   1 t(4) t(4)^2 t(4)^3 t(4)^4 t(4)^5;
   0   1  2*t(1) 3*t(1)^2 4*t(1)^3 5*t(1)^4;
   0   1  2*t(4) 3*t(4)^2 4*t(4)^3 5*t(4)^4;
    0   1  2*t(2) 3*t(2)^2 4*t(2)^3 5*t(2)^4;
    0   1  2*t(3) 3*t(3)^2 4*t(3)^3 5*t(3)^4];
   

% la ecuacion es:
% A*k = b
% donde b es un vector de las condiciones de contorno
% posicion inicial y final
% velocidad inicial y final
% aceleraciones inicial y final

b(1) = q(1);
b(2) = q(2);

b(3) = 0;
b(4) = 0;

b(5) = qdmax;
b(6) = qdmax;


%k = [k(1) k(2) k(3) k(4) k(5) k(6)];

k = inv(A)*b(:);


time = t(1):delta_t:t(4);

q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3 + k(5)*time.^4 + k(6)*time.^5;

qd_t= k(2) + 2*k(3)*time + 3*k(4)*time.^2 + 4*k(5)*time.^3 + 5*k(6)*time.^4;

qdd_t=2*k(3) + 6*k(4)*time + 12*k(5)*time.^2 + 20*k(6)*time.^3;

plot(time, q_t), hold
plot(time, qd_t)
plot(time, qdd_t)
