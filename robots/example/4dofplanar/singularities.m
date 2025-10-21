%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find singularities in vx vy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=singularities
syms q1 q2 q3 q4 real
L1=1;
L2=1;
L3=1;
L4=1;
j11 = -(L1*sin(q1) + L2*sin(q1+q2) + L3*sin(q1+q2+q3) + L4*sin(q1+q2+q3+q4));
j12 = -(L2*sin(q1+q2) + L3*sin(q1+q2+q3) + L4*sin(q1+q2+q3+q4));
j13 = -(L3*sin(q1+q2+q3) + L4*sin(q1+q2+q3+q4));
j14 = -(L4*sin(q1+q2+q3+q4));

j21 = (L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3) + L4*cos(q1+q2+q3+q4));
j22 = (L2*cos(q1+q2) + L3*cos(q1+q2+q3) + L4*cos(q1+q2+q3+q4));
j23 = (L3*cos(q1+q2+q3) + L4*cos(q1+q2+q3+q4));
j24 = (L4*cos(q1+q2+q3+q4));

J = [j11 j12 j13 j14; j21 j22 j23 j24];
JJ = J*J';

JJ = simplify(JJ)

dJJ=det(JJ)

dJJ = simplify(dJJ)

q1 = pi/4
q2 = 0
q3 = 0
q4 = pi/4

dJJ = eval(dJJ)

%solve(dJJ==0)


