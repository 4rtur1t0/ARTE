%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK MCL 
% Ik mcl tries to find all posible solutions to the inverse kinematic
% problem that allow to reach the same position/orientation in space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=ik_null_space_4dofplanar
close all;
%global robot
robot = load_robot('example', '4dofplanar');

step_time = 0.1;
n_movements = 300;
q0 = pi/8*[1 1 1 1]';
signo = 1;
a=1;
qq = [];
drawrobot3d(robot, q0)
adjust_view(robot)
q =  q0; % inverse_kinematics_4dofplanar(robot, T0, q0);

%hold
for i=1:n_movements
    fprintf('Move %d out of %d\n', i, n_movements)  
    qd = null_space_4dof(robot, q);
    q = q + signo*qd*step_time;
    qq = [qq q];
    
    mod_qd = norm(qd);
    if mod_qd < 5e-3 && a
        signo = -1;
        a=0;
    end
end
animate(robot, qq)



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

function T = build_T(p, phi)
T = [cos(phi) -sin(phi) 0 p(1);
     sin(phi) cos(phi) 0 p(2);
     0            0     1  p(3);
     0             0    0   1];

 
