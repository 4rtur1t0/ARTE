%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Produce a self motion for a 4 DOF planar robot.
% Exercise: 
% complete the function null_space_4dof below.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function null_space_kuka_lbr_iiwa
close all
robot = load_robot('KUKA','LBR_IIWA_R820_7DOF')
q = pi/4*[1 1 1 1 1 1 1]';
J = manipulator_jacobian(robot, q);
% m=6
% n=7

qs = [];
qds = [];
for i=1:150
   J = manipulator_jacobian(robot, q);
   Jd = pinv(J);
   q0 = [0 0 1 0 0 0 0]';
   qd = (eye(7)-Jd*J)*q0;
   q = q + qd*0.05;
   qs = [qs q];
   qds = [qds qd];
end
figure, plot(qs', 'Linewidth', 3), legend('1', '2', '3', '4', '5', '6', '7'), title('joint positions')
figure, plot(qds', 'Linewidth', 3), legend('1', '2', '3', '4', '5', '6', '7'), title('speeds')
animate(robot, qs)