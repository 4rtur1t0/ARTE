function [qt1,qdt1,qddt1,time1] = interpola3(delta_t,qi,qf,t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%INTERPOLADOR DE 3ER ORDEN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
amax = 2; %rad/s/s
wmax = 1; %rad/s
t=[0 t];

q=[qi qf];


[qt1,qdt1,qddt1,time1,k1]=interpola3orden(q,  [0 wmax], [0 amax], t, delta_t);











function [q_t, qd_t, qdd_t, time, k]=interpola3orden(q, qd, qdd, t, delta_t)

A=[1 t(1) t(1)^2 t(1)^3;
   1 t(2) t(2)^2 t(2)^3;
   0   1  2*t(1) 3*t(1)^2;
   0   0   2     6*t(1)];
k=inv(A)*[q(1) q(2) qd(1) qdd(1)]';

time=t(1):delta_t:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3;
qd_t= k(2)*ones(1,length(time)) + 2*k(3)*time + 3*k(4)*time.^2;
qdd_t=2*k(3)*ones(1,length(time)) + 6*k(4)*time;
