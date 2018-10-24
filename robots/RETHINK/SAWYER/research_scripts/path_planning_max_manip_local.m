% Max manipulability index.
% Start by maximizing manipulability locally at each time step
function path_planning_max_manip_local%(robot)
close all;
global robot
%movement in meters from start pose
deltaV = [0 1.25 0]';
ds=0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial and end poses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0 = [-1.3 -0.6 0.0 1.2 0.3 1.0 0.3]';
T0=directkinematic(robot, q0);
p0 = T0(1:3,4);

%The inverse kinematics is solved for the following reason
%if using the moore penrose
% this allows the joint positions to be continuous
%can be solved with moore_penrose or with q3_0
q = inverse_kinematics_sawyer(robot, T0, q0, 'moore_penrose');
q = optimize_manip_local(robot, q, 'max_manip');
drawrobot3d(robot, q)
draw_axes(T0, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
pp0 = [p0 p0 + deltaV];
plot3(pp0(1,:),pp0(2,:),pp0(3,:),'k')

mov = ds:ds:norm(deltaV);
Ti=T0;
%q = q0;
%initial pose and manipulability
qq = [q];
pp = [];
pph = [];
for i=1:length(mov)    
    fprintf('Move %d out of %d\n', i, length(mov))
    %update to next point in trajectory
    Ti(1:3,4) = p0 + mov(i)*deltaV;
    
    %draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    %solve inverse kinematics from starting q
    %q = pinv(J)*X
    q = inverse_kinematics_sawyer(robot, Ti, q, 'moore_penrose');
    %max manipulability along the null space at that particular pose
    q = optimize_manip_local(robot, q, 'max_manip');
    Th=directkinematic(robot, q);
    %is q continuous?
    drawrobot3d(robot, q)
    draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    plot3(pp0(1,:),pp0(2,:),pp0(3,:),'k')
    qq = [qq q];
    pp = [pp Ti(1:3,4)];
    pph = [pph Th(1:3,4)];
end
%det(J*J')
manips = compute_manip(robot, qq);

%for i=1:size(qq,2)
%     drawrobot3d(robot, qq(:,i))
%     draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);     
%end
plot3(pp(1,:),pp(2,:),pp(3,:))
plot3(pph(1,:),pph(2,:),pph(3,:),'r')
figure,
plot(manips)
title('manipulability index at each movement')

figure,
plot(qq')
title('joint positions')
legend('q_1', 'q_2','q_3','q_4','q_5','q_6','q_7')

'max manipulability'
max_manip = max(manips)

'min manipulability'
min_manip = min(manips)

'mean manipulability'
mean_manip = mean(manips)

'semi sum manipulability'
semi_manip = (max_manip-min_manip)/2

save test_no_max_manip.mat



function manips = compute_manip(robot, qs)
manips = [];
for i=1:size(qs,2)
    %compute current manip
    J = manipulator_jacobian(robot, qs(:,i));
    manip = det(J*J');
    manips = [manips manip];
end





function animate_local(robot, q)
global configuration 

h=figure(configuration.figure.robot);, hold on,

%get adjusted view
[az,el] = view;
for j=1:size(q, 2),
    clf(h);
    qj=q(:,j);  
    view(az,el);
   
    drawrobot3d(robot, qj);  
    
    %pause to get a nice view
    pause(0.1);   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_orientation(Qf, Qi)
Q = Qf-Qi;
reach = sqrt(Q(1)^2 + Q(2)^2 + Q(3)^2 + Q(4)^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_position(Pf, Pi)
P = Pf-Pi;
reach = sqrt(P(1)^2 + P(2)^2 + P(3)^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% normalize vector if possible.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function v = normalize(v)
d = sqrt(v(1)^2+v(2)^2+v(3)^2);
if d>0
    v = v/d;
end
v=v(:);
