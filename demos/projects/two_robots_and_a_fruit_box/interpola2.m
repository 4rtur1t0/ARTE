function [q_t,qd_t,time]=interpola2(delta_t,qi,qf,t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%INTERPOLADOR DE 1ER ORDEN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=[qi qf];

T1 = 0;
T2 = t;
t=[T1 T2];



 
[q_t, qd_t, time, k]=first_order([q(1) q(2)], [t(1) t(2)], delta_t); 




function [q_t, qd_t, time, k]=first_order(q, t, delta_t)

A = [1 t(1); 
     1 t(2)];
b = [q(1) q(2)];

k=inv(A)*b';


time=t(1):delta_t:t(2);

q_t=k(1) + k(2)*time;

qd_t=k(2)*ones(1,length(time));