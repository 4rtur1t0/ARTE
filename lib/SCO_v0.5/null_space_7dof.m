% Compute null space vanilla
function [qd_null] = null_space_7dof(robot, q)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
I = eye(robot.DOF);
Jp=pinv(J);
%null space projector
%n_space_projector = (I-Jp*J);
%for an arbitrary vector
%do not use [1 1 1 1]
qd3 = [0 0 1 0 0 0 0]';
qd_null = project(J, Jp, I, qd3);

function qd_null = project(J, Jp, I, qd)
%q2 está calculado a través de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
qd_null = (I-Jp*J)*qd;

 
