%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find singularities in vx vy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=singularities
syms q1 q2 q3 real
L1=1;
L2=1;
L3=1;
j11 = -(L1*sin(q1) + L2*sin(q1+q2));
j12 = -L2*sin(q1+q2);

j21 = L1*cos(q1) + L2*cos(q1+q2);
j22 = L2*cos(q1+q2);

%J = [j11 j12; j21 j22];
J = [j11 j12];

JJ = J*J';

%JJ = simplify(JJ)

dJJ=det(JJ)

%dJJ = simplify(dJJ)

%q1 = 0
%q2 = 0
 
%dJJ = eval(dJJ)
eqn = dJJ==0
S=solve(eqn, [q1 q2])


