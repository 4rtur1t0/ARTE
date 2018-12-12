%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability index ALONG A LINE.
% Start by maximum global manipulability at a pose
% Start by maximizing manipulability locally at each time step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=path_planning_local_max%(direction)
close all;
global robot

experiment_name = 'min_manip_local.mat';

height1 = 2; %m
height2 = 2.5; %m
x1 = -1;
x2 = 2;

%the orientation needed
phi = pi/2; 
p0 = [x1 height1 0]';
pf = [x2 height2 0]';

T0 = build_T(p0, phi);
Tf = build_T(pf, phi);

%this defines the line to follow in direction
deltaV = pf - p0;
deltaV = deltaV/norm(deltaV);
%and total length of the movement
total_length = norm(pf - p0); %m
%the movement step in m
ds=0.1;
mov = 0.0:ds:total_length;

line0 = [p0 pf];
plot(line0(1,:),line0(2,:),'k')

%initial pose and manipulability
qq = [];

q0 = [pi/2 -pi -pi pi/4]';
q = q0;
Ti = T0;
for i=1:length(mov)    
    fprintf('Move %d out of %d\n', i, length(mov))
    %update to next point in trajectory
    Ti(1:3,4) = p0 + mov(i)*deltaV;   
    %(robot,  Tf, q0, restriction)
    q = inversekinematic_4dofplanar(robot, Ti, q);
    drawrobot3d(robot, q)
    q = max_manipulability_local(robot, q, -1);
    %drawrobot3d(robot, q)
    draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    plot3(line0(1,:),line0(2,:),line0(3,:),'k')
    qq = [qq q];
end

animate_local(robot, qq, line0)

manips = compute_manip(robot, qq);
figure,
plot(manips)
title('manipulability index at each movement')
figure,
plot(qq')
title('joint positions')
legend('q_1', 'q_2','q_3','q_4','q_5','q_6','q_7')

save(experiment_name)

% Simple iteration along the null space to increase manipulability
function [q] = max_manipulability_local(robot, q, sign_max)

delta_time = 0.01;
while 1
%    [qd_null] = compute_null_space(robot, q);
[qd_null] = null_space_4dof(robot, q);
    delta_manip = compute_delta_manip(robot, q, qd_null, 0.01);
    % add a sign for maximization/minimization
    % the constant is based on the rate of delta_manip also
    x = sign_max*sign(delta_manip)*qd_null;
    q = q + x*delta_time;
    if abs(delta_manip) < 1e-2
        break
    end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computes a gradient so as to let the manipulability be improved
% q is the current joint position
% qd is the instantaneous speed that lies in the null space
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_manip = compute_delta_manip(robot, q, qd, delta)
%compute first manipulability index
m0 = compute_manip(robot, q);
%move differentially along the null space.
q = q + delta*qd;
%compute second manipulability index
m1 = compute_manip(robot, q);
%delta_manip = trace(inv(J*J')*(Jd*J'+J*Jd'));
%return difference
delta_manip=(m1-m0)/delta;

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

function animate_local(robot, qq, line_work)
for i=1:1:size(qq,2)
     drawrobot3d(robot, qq(:,i))    
     plot(line_work(1,:),line_work(2,:))
end

 
