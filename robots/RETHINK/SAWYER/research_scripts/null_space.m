%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NULL SPACE PART A
% FORWARD kinematics
% at a given joint position, is there a set of
% joint speeds that do not produce any speed at the end effector.
%   yes! that is the null space.
% that is an ego-motion that the robot can still do without affecting
% the end effector position or orientation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%q0 = [0.0 -0.6 0.0 1.2 0.0 1.0 0.0]';
q = [0.0 -0.4 0.1 1.0 0.1 0.1 0.1]';


%compute nil-space?
J = manipulator_jacobian(robot, q);
I = eye(7);
Jp=penrose(J);
%null space projector
n_space = (I-Jp*J);
%V = [0 0 0 0 0 0]';
%y = Jp*V
%n_space*u



%para un vector arbitrario
qd = [31 1 51 14 31 31 1]';
%calculamos la primera solución usando la pseudoinversa
q1 = Jp*V;
%q2 está calculado a través de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
q2 = (I-Jp*J)*qd;

%ESTE NULL space también se puede calcular con matlab usando una
%descomposición SVD
nn = null(J);
%q2 y nn son paralelos
q2/norm(q2)-nn

%ojo, q2, no produce ningún movimiento en el extremo
%q2 es solamente un self-motion, posible
% de tal manera que V_e = J*q1 y J*q2=0
V_e = J*q1 + J*q2

%en lo anterior, x1 es la primera solución de la primera restricción
%x2 es la solución después la segunda restricción
% es obvio que la diferencia entre las dos es un self-motion
qd0 = (x1 - x2)
%este self motion no produce movimiento en el extremo J*qd0
u = J*x2 + J*qd0;