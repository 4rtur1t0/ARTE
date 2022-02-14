function path_planning_line_2dof_planar_improved()
close all    
    errors1 = path_planning_vanilla()
    errors2 = path_planning_corrected_v()
    errors3 = path_planning_corrected_nv()

    figure, plot(errors1,'Linewidth', 3 ), hold
    plot(errors2,'Linewidth', 3 ), 
    plot(errors3,'Linewidth', 3 ), 
    xlabel('time step'), ylabel('Mínimo error recta (m)')
end

function errors = path_planning_vanilla()

robot = load_robot('example', '2dofplanar');
delta_time = 0.01; % s
abs_linear_speed = 1.5; %m/s
% initial point
T1=[1 0 0 0.8;
    0 1 0 -0.5;
    0 0 1 0; 
    0 0 0  1];
%end point
T2=[1 0 0 -0.5;
    0 1 0 1.5;
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
errors = [];
while 1
   T = directkinematic(robot, q);
   J = manipulator_jacobian(robot, q);
   J = J(1:2,:);
   p = T(1:3,4);
   %error = norm(end_point - p);
   [error_end, error_line, error_line_vector]=find_errors(start_point, end_point, p);
   if error_end < 0.05       
       break
   end
   qd = inv(J)*v(1:2);
   q = q + qd*delta_time;
   qs = [qs q]; qds = [qds q]; ps = [ps p];
   errors = [errors error_line]
end
% figure, plot(qs', 'Linewidth', 3), legend('q_1', 'q_2')
% figure, plot(qds','Linewidth', 3), legend('qd_1', 'qd_2')
% figure, plot(ps(1,:), ps(2, :),'Linewidth', 3 ), xlabel('p_x'), ylabel('p_y')
% figure, plot(errors,'Linewidth', 3 ), xlabel('time step'), ylabel('Mínimo error recta (m)')
% animate(robot, qs)
end

function errors = path_planning_corrected_v()

robot = load_robot('example', '2dofplanar');
delta_time = 0.01; % s
abs_linear_speed = 1.5; %m/s
% initial point
T1=[1 0 0 0.8;
    0 1 0 -0.5;
    0 0 1 0; 
    0 0 0  1];
%end point
T2=[1 0 0 -0.5;
    0 1 0 1.5;
    0 0 1 0; 
    0 0 0  1];

start_point = T1(1:3,4);
end_point = T2(1:3,4);
% vector velocidad en la direccion de la trayectoria
%v = (end_point-start_point);
%v = abs_linear_speed*v/norm(v); %vector normalizado en la direcci�n de la recta

% Solve inverse kinematics at first position
% caution, the orientation in T1 is not achieved
qinv = inversekinematic(robot, T1);

q = qinv(:,2);

qs = [];
qds = [];
ps = [];
errors = [];
while 1
   T = directkinematic(robot, q);
   J = manipulator_jacobian(robot, q);
   J = J(1:2,:);
   p = T(1:3,4);
   
   [error_end, error_line, error_line_vector]=find_errors(start_point, end_point, p);
   if error_end < 0.01       
       break
   end
   % recompute v at each time step
   % norm(v) cannot be null here!
   v = (end_point - p);
   v = abs_linear_speed*v/norm(v);   
   qd = inv(J)*v(1:2);
   q = q + qd*delta_time;
   qs = [qs q]; qds = [qds q]; ps = [ps p];
   errors = [errors error_line]
end
% figure, plot(qs', 'Linewidth', 3), legend('q_1', 'q_2')
% figure, plot(qds','Linewidth', 3), legend('qd_1', 'qd_2')
% figure, plot(ps(1,:), ps(2, :),'Linewidth', 3 ), xlabel('p_x'), ylabel('p_y')
% figure, plot(errors,'Linewidth', 3 ), xlabel('time step'), ylabel('Mínimo error recta (m)')
%animate(robot, qs)


end

function errors = path_planning_corrected_nv()

robot = load_robot('example', '2dofplanar');
delta_time = 0.01; % s
abs_linear_speed = 1.5; %m/s
% initial point
T1=[1 0 0 0.8;
    0 1 0 -0.5;
    0 0 1 0; 
    0 0 0  1];
%end point
T2=[1 0 0 -0.5;
    0 1 0 1.5;
    0 0 1 0; 
    0 0 0  1];

start_point = T1(1:3,4);
end_point = T2(1:3,4);
% vector velocidad en la direccion de la trayectoria
%v = (end_point-start_point);
%v = abs_linear_speed*v/norm(v); %vector normalizado en la direcci�n de la recta

% Solve inverse kinematics at first position
% caution, the orientation in T1 is not achieved
qinv = inversekinematic(robot, T1);

q = qinv(:,2);

qs = [];
qds = [];
ps = [];
errors = [];
while 1
   T = directkinematic(robot, q);
   J = manipulator_jacobian(robot, q);
   J = J(1:2,:);
   p = T(1:3,4);
   
   [error_end, error_line, error_line_vector]=find_errors(start_point, end_point, p);
   if error_end < 0.01       
       break
   end
   % decompose v into parallel and normal direction
   [v1, v2] = v_decomposition(start_point, end_point, p);
    v1 = abs_linear_speed*v1/norm(v1);
    v2 = 1.5*v2/delta_time;
    v = v1 + v2; 
   %v = abs_linear_speed*v/norm(v);  
   qd = inv(J)*v(1:2);
   q = q + qd*delta_time;
   qs = [qs q]; qds = [qds q]; ps = [ps p];
   errors = [errors error_line];
end
figure, plot(qs', 'Linewidth', 3), legend('q_1', 'q_2')
figure, plot(qds','Linewidth', 3), legend('qd_1', 'qd_2')
figure, plot(ps(1,:), ps(2, :),'Linewidth', 3 ), xlabel('p_x'), ylabel('p_y')
figure, plot(errors,'Linewidth', 3 ), xlabel('time step'), ylabel('Mínimo error recta (m)')
animate(robot, qs)


end

function [v1, v2] = v_decomposition(a, b, p)

% parallel direction
vp = (b-a);
vp = vp/norm(vp);
% normal direction, alredy unitary
vn = [-vp(2) vp(1) 0]';

% error speed
p = (b-p);
%p = p/norm(p);

% project the speed to both directions
v1 = vp'*p*vp;
v2 = vn'*p*vn;


end


% find:
% error_end: the error with respect to the end point
% error_line: the error of p to the line defined b point a and vector n.
% the line is defined by points a and b.
function [error_end, error_line, error_line_vector]=find_errors(a, b, p)
error_end = sqrt((p-b)'*(p-b));

% define the line as a, n
n = (b-a);
n = n/norm(n);

error_line = (a-p)-((a-p)'*n)*n;
error_line = norm(error_line);

% Now obtain current point minus initial point a
% project
w = p-a;
p_ = n*dot(w, n);
% add the origin since the point p_ is referred to the line
p_ = p_ + a;
% find a vector connecting the current point p and the point belonging to
% the line p_
error_line_vector = p_ - p;
end


