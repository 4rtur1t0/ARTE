%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Produce a self motion for a 4 DOF planar robot.
% Exercise: 
% complete the function null_space_4dof below.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=self_motion
close all;
global robot

robot = load_robot('example','4dofplanar')
%particles
M = 1500;
step_time = 0.5;

q0 = [0 0.5 0.5 0.5]';
T0 = directkinematic(robot, q0);
signo = 1;
a=1;
%initial pose and manipulability
qq = [];

q0 = [0.1 0.1 0.1 0.1]';
drawrobot3d(robot, q0)
adjust_view(robot)
q = inverse_kinematics_4dofplanar(robot, T0, q0);

for i=1:M
    fprintf('Move %d out of %d\n', i, M)  
    qd = null_space_4dof(robot, q);
    q = q + signo*qd*step_time;
    drawrobot3d(robot, q)
    pause(0.1)
    draw_axes(T0, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    qq = [qq q];
end


%
% Computes manipulability of poses given in qs
% Caution: only taking into account vx, vy and wz
%
function manips = compute_manip(robot, qs)
manips = [];
for i=1:size(qs,2)
    %compute current manip
    J = manipulator_jacobian(robot, qs(:,i));
    J = [J(1:2,:); J(6,:)];
    manip = sqrt(det(J*J'));
    manips = [manips manip];
end

function qd=null_space_4dof(robot, q)
%J=...
%I-J'J--> compute projection


 
