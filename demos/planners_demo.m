%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THIS DEMO PRESENTS THREE DIFFERENT JOINT PLANNERS.
%   - A first order planner.
%   - A second order planner.
%   - A third order planner.
%
%   Every planner is applied to a single joint.
%   The planner must "plan" (i.e. compute) a trajectory starting at q(1),
%   going through q(2) and ending at q(3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function planners_demo
close all

% three points for the planner in position
q=[0 0.5 0.8];
%starting speed
qd=[0];
%starting acceleration
qdd=[0];
% time vector in seconds
t=[0 10 20];
%the minimum difference in seconds between times to compute q(t) functions
delta_t=0.001;

%FIRST ORDER PLANNER PLAN TWO DIFFERENT TRAJECTORIES FROM q1 to q2 and from
%q2 to q3.
figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('FIRST ORDER PLANNER'), hold on
[time, q_t, qd_t, k]=first_order([q(1) q(2)],[t(1) t(2)], delta_t); plot(time,q_t, 'r'), plot(time,qd_t, 'g')
[time, q_t, qd_t, k]=first_order([q(2) q(3)],[t(2) t(3)], delta_t); plot(time,q_t, 'r'), plot(time,qd_t, 'g')
legend('Joint position q (rad)', 'Joint speed q (rad/s)')

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('SECOND ORDER PLANNER'), hold on
[k,qd_end]=second_order([q(1) q(2)], qd(1),[t(1) t(2)], delta_t);
k=second_order([q(2) q(3)],qd_end,[t(2) t(3)], delta_t);

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('THIRD ORDER PLANNER'), hold on
[k,qd_end,qdd_end]=third_order([q(1) q(2)],qd(1),qdd(1),[t(1) t(2)], delta_t);
k=third_order([q(2) q(3)],qd_end,qdd_end,[t(2) t(3)], delta_t);

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('THIRD ORDER, SECOND VERSION'), hold on
%the start and end speeds are given
[k]=third_order_2([q(1) q(3)],[qd(1) qd(1)],[t(1) t(3)], delta_t);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First order planner.
%   Given the start and end joint positions and times compute a first order
%   polinomial of the form:
%   q(t) = k(1) + k(2)*t
%
%   Returns:
%       the time vector used.
%       the values of q(t) as a function of the time vector used.
%       the speed qd(t) as a function of the time vector (which is a constant in this case)
%       the polynomial coefficients k that allow to compute q(t) as:
%       q(t) = k(1) + k(2)*t
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t, q, qd, k]=first_order(q_i, t_i, delta_t)
% compute the coefficients of k for a first order planner
k=inv([1 t_i(1); 1 t_i(2)])*[q_i(1) q_i(2)]';

t=t_i(1):delta_t:t_i(2);
q=k(1) + k(2)*t;
qd=k(2)*ones(1,length(t));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Second order planner
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t, q, qd,  k,qd1]=second_order(q_i, qd_i, t_i, delta_t)

k=inv([1 t(1) t(1)^2; 1 t(2) t(2)^2; 0 1 2*t(1)])*[q_i(1) q_i(2) qd_i(1)]';

t=t_i(1):delta_t:t_i(2);
q=k(1)+k(2)*t+k(3)*t.^2;
qd_t=k(2)*ones(1,length(t)) + 2*k(3)*t;
qdd_t=2*k(3)*ones(1,length(t));
qd1 = k(2) + 2*k(3)*t(2);

plot(time,q_t, 'r')
plot(time,qd_t, 'g')
plot(time,qdd_t, 'b')


%   third order, first version
function [k,qd1, qdd1]=third_order(q,qd,qdd,t)
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


%   third order, second version
function [k,qd1, qdd1]=third_order_2(q,qd,t)
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
