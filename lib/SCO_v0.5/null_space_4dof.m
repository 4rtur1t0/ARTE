% Compute null space vanilla
function [qd_null] = null_space_4dof(robot, q)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
% moving on vx, vy and wz!!!
%J = [J(1:2,:); J(6,:)];
J = J(robot.selJ, :);
I = eye(robot.DOF);
Jp=pinv(J);
%null space projector
%n_space_projector = (I-Jp*J);
%for an arbitrary vector
%do not use [1 1 1 1]
qd = [1 0 0 0]';
qd_null = (I-Jp*J)*qd;


 
