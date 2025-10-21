% Simple Path Planning of a 3DOF planar robot
function path_planning_line_3dof_planar_jacobian()
close all
robot = load_robot('example', '3dofplanar');
delta_time = 0.05; % s
% max linear speed and accel
v_max = 1.0; %m/s
a_max= 2; %m/s/s
% max angular speed and accel
w_max = 2; %rad/s
alpha_max = 2; %rad/s


% % initial point
% T1=[1 0 0 0.8;
%     0 1 0 -0.5;
%     0 0 1 0; 
%     0 0 0  1];
% %end point
% T2=[1 0 0 -0.5;
%     0 1 0 1.5;
%     0 0 1 0; 
%     0 0 0  1];

% initial point
T1=[1 0 0 0.8;
    0 1 0 -0.2;
    0 0 1 0; 
    0 0 0  1];
%end point
T2=[1 0 0 -0.8;
    0 1 0 0.2;
    0 0 1 0; 
    0 0 0  1];


start_point = T1(1:3,4);
end_point = T2(1:3,4);
% vector velocidad en la direccion de la trayectoria
v = (end_point-start_point);
v = abs_linear_speed*v/norm(v); %vector normalizado en la direcci�n de la recta

% Solve inverse kinematics at first position
% caution, the orientation in T1 is not achieved
qinv = inversekinematic(robot, T1);

q = qinv(:,2);

qs = [];
qds = [];
ps = [];
% while 1
for i=1:1000
    
   T = directkinematic(robot, q);
   J = manipulator_jacobian(robot, q);
   J = J(1:2,:);
   p = T(1:3,4);
   error = norm(end_point - p);
   if error < 0.01     
       break
   end
   qd = inv(J)*v(1:2);
   q = q + qd*delta_time;
   qs = [qs q];
   qds = [qds qd];
   ps = [ps  p];   
end
figure, plot(qs', 'Linewidth', 3), legend('q_1', 'q_2')
figure, plot(qds','Linewidth', 3), legend('qd_1', 'qd_2')
figure, plot(ps(1,:), ps(2, :),'Linewidth', 3 ), xlabel('p_x'), ylabel('p_y')
animate(robot, qs)

