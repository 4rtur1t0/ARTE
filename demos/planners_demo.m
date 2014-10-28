function planners_demo
close all
q0=0;
qd0=0;
qdd0=0;
q1=0.5; %rad
q2=0.8;
t0=0;
t1=10;%s
t2=20;%s

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('PRIMER ORDEN'), hold on


k=first_order([q0 q1],[t0 t1]);
k=first_order([q1 q2],[t1 t2]);


figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('SEGUNDO ORDEN'), hold on


[k,qd1]=second_order([q0 q1],qd0,[t0 t1]);
k=second_order([q1 q2],qd1,[t1 t2]);


figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('TERCER ORDEN'), hold on
[k,qd1,qdd1]=third_order([q0 q1],qd0,qdd0,[t0 t1]);
k=third_order([q1 q2],qd1,qdd1,[t1 t2]);

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('TERCER ORDEN, SEGUNDA VERSIÓN'), hold on
[k]=third_order_2([q0 q2],[0 0],[t0 t2]);





%First order planner
function k=first_order(q,t)

%   PRIMER ORDEN

k=inv([1 t(1); 1 t(2)])*[q(1) q(2)]';

t_v=t(1):0.01:t(2);
q_v=k(1)+k(2)*t_v;
qd_v=k(2)*ones(1,length(t_v));

plot(t_v,q_v, 'r')
plot(t_v,qd_v, 'g')


%Second order planner
function [k,qd1]=second_order(q,qd,t)


k=inv([1 t(1) t(1)^2; 1 t(2) t(2)^2; 0 1 2*t(1)])*[q(1) q(2) qd(1)]';

time=t(1):0.01:t(2);
q_t=k(1)+k(2)*time+k(3)*time.^2;
qd_t=k(2)*ones(1,length(time)) + 2*k(3)*time;
qdd_t=2*k(3)*ones(1,length(time));

qd1 = k(2) + 2*k(3)*t(2);

plot(time,q_t, 'r')
plot(time,qd_t, 'g')
plot(time,qdd_t, 'b')



function [k,qd1, qdd1]=third_order(q,qd,qdd,t)

%   tercer orden
A=[1 t(1) t(1)^2 t(1)^3;
   1 t(2) t(2)^2 t(2)^3;
   0   1  2*t(1) 3*t(1)^2;
   0   0   2     6*t(1)];
k=inv(A)*[q(1) q(2) qd(1) qdd(1)]';

time=t(1):0.01:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3;
qd_t= k(2)*ones(1,length(time)) + 2*k(3)*time + 3*k(4)*time.^2;
qdd_t=2*k(3)*ones(1,length(time)) + 6*k(4)*time;

qd1 = k(2) + 2*k(3)*t(2) + 3*k(4)*t(2).^2;
qdd1 = 2*k(3) + 6*k(4)*t(2);

plot(time,q_t, 'r')
plot(time,qd_t, 'g')
plot(time,qdd_t, 'b')


function [k,qd1, qdd1]=third_order_2(q,qd,t)

%   tercer orden
A=[1 t(1) t(1)^2 t(1)^3;
   1 t(2) t(2)^2 t(2)^3;
   0   1  2*t(1) 3*t(1)^2;
   0   1  2*t(2) 3*t(2)^2];
k=inv(A)*[q(1) q(2) qd(1) qd(2)]';

time=t(1):0.01:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3;
qd_t= k(2)*ones(1,length(time)) + 2*k(3)*time + 3*k(4)*time.^2;
qdd_t=2*k(3)*ones(1,length(time)) + 6*k(4)*time;

qd1 = k(2) + 2*k(3)*t(2) + 3*k(4)*t(2).^2;
qdd1 = 2*k(3) + 6*k(4)*t(2);

plot(time,q_t, 'r')
plot(time,qd_t, 'g')
plot(time,qdd_t, 'b')
