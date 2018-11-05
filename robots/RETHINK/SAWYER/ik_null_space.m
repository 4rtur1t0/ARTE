%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IK MCL 
% Ik mcl tries to find all posible solutions to the inverse kinematic
% problem that allow to reach the same position/orientation in space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=ik_null_space
close all;
global robot

%particles
M = 1500;
step_time = 0.5;

height1 = 2; %m
x1 = -1;

%the orientation needed
phi = pi/2; 
p0 = [x1 height1 0]';
T0 = build_T(p0, phi);
signo = 1;
a=1;
%initial pose and manipulability
qq = [];
q0 = [0 0 0 0 0 0 0]';
q = inversekinematic(robot, T0, q0);
%hold
for i=1:M
    fprintf('Move %d out of %d\n', i, M)  
    qd = null_space_7dof(robot, q);
    q = q + signo*qd*step_time;
    drawrobot3d(robot, q)
    pause(0.1)
    draw_axes(T0, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    qq = [qq q];
    
    mod_qd = norm(qd);
    if mod_qd < 1e-3 && a
        signo = -1;
        a=0;
    end
    manips = compute_manip(robot, qq);
    figure(2)
    plot(manips)
    
end

manips = compute_manip(robot, qq);
figure,
plot(manips)
title('manipulability index at each movement')

%plot correlation between variables
%using covariance matrix
A = cov(qq');
colormap('hot')
imagesc(A)
colorbar

qq_sorted = sortrows(qq');
qq_sorted = qq_sorted';

save(experiment_name)

for i=1:1:size(qq,2)
     drawrobot3d(robot, qq_sorted(:,i))    
     %plot(line_work(1,:),line_work(2,:))
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

function T = build_T(p, phi)
T = [cos(phi) -sin(phi) 0 p(1);
     sin(phi) cos(phi) 0 p(2);
     0            0     1  p(3);
     0             0    0   1];

 
