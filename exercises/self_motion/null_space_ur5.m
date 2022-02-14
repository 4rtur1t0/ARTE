%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Produce a self motion for a 4 DOF planar robot.
% Exercise: 
% complete the function null_space_4dof below.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function null_space_ur5
close all
%robot = load_robot('ABB','IRB140')
robot = load_robot('UR','UR5')
q = pi/8*[1 1 1 1 1 1]';
q = pi/8*[0 0 0 0 0 0]';

drawrobot3d(robot, q)
% m=3
% n=6
qs = [];
qds = [];
for i=1:50
   J = manipulator_jacobian(robot, q);
   % m=3, axis=4 (omegax)
   qd = move_along_axis(J, 3, 4);   
   %qd = move_along_axis(J, 6, 4);   
   q = q + qd*0.05;
   qs = [qs q];
   qds = [qds qd];
   % what happens
   vw = J*qd;   
end
figure, plot(qs', 'Linewidth', 3), legend('1', '2', '3', '4', '5', '6', '7'), title('joint positions')
figure, plot(qds', 'Linewidth', 3), legend('1', '2', '3', '4', '5', '6', '7'), title('speeds')
animate(robot, qs)


function qd= move_along_axis(J, m, axis)
J = J(1:m, :)
[U,S,V] = svd(J)
qd = V(:,axis) 


