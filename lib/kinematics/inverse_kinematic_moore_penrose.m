
%   INVERSE KINEMATICS in speed using the Moore-Penrose pseudo invers
function qd = inverse_kinematic_moore_penrose(robot, q, Vref)
J = manipulator_jacobian(robot, q);
Jp = pinv(J);
qd = Jp*Vref;