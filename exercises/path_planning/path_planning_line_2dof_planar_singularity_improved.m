%
% Different strategies to test when the robot goes into singularity
%
function path_planning_line_2dof_planar_singularity_improved()
   
    errors = path_planning_corrected_nv()

    figure, 
    plot(errors,'Linewidth', 3 ), 
    xlabel('time step'), ylabel('Mínimo error recta (m)')
end


function errors = path_planning_corrected_nv()

robot = load_robot('example', '2dofplanar');
delta_time = 0.01; % s
abs_linear_speed = 1.5; %m/s
% % initial point
% T1=[1 0 0 1.5;
%     0 1 0 0;
%     0 0 1 0; 
%     0 0 0  1];
% %end point
% T2=[1 0 0 -1.5;
%     0 1 0 0;
%     0 0 1 0; 
%     0 0 0  1];
% initial point
T1=[1 0 0 2;
    0 1 0 0.0;
    0 0 1 0; 
    0 0 0  1];
%end point
T2=[1 0 0 -2;
    0 1 0 0.0;
    0 0 1 0; 
    0 0 0  1];


start_point = T1(1:3,4);
end_point = T2(1:3,4);

% Solve inverse kinematics at first position
% caution, the orientation in T1 is not achieved
qinv = inversekinematic(robot, T1);

q = qinv(:,2);

qs = [];
qds = [];
ps = [];
errors = [];
% real, end effector speed
vs = [];
qd = [0 0]';
%while 1
for i=1:1500
   T = directkinematic(robot, q);
   J = manipulator_jacobian(robot, q);
   p = T(1:3,4);
   
   [error_end, error_line, error_line_vector]=find_errors(start_point, end_point, p);
   if error_end < 0.01       
       break
   end
   % decompose v into parallel and normal direction
   % [v1, v2] = v_decomposition(start_point, end_point, p);
   % v1 = abs_linear_speed*v1/norm(v1);
   % v2 = 1.5*v2/delta_time;
   % v = v1 + v2;
   v = (end_point-p);
   v = 1.5*v/norm(v);
   %v = v1;
   % qd = jacobian_control_vanilla(robot, q, v(1:2));
   %qd = jacobian_control_damped(robot, q, v(1:2));
   qd = jacobian_control_svd3(robot, q, qd, v(1:2));
   qd = qd/norm(qd);   
   q = q + qd*delta_time;
   qs = [qs q]; qds = [qds q]; ps = [ps p]; 
   vs = [vs J*qd];
   errors = [errors error_line];
   %drawrobot3d(robot, q)
   %pause(0.01)
end
figure, plot(vs(1,:)), hold, plot(vs(2,:)), legend('v_x', 'v_y')
figure, plot(qs', 'Linewidth', 3), legend('q_1', 'q_2')
figure, plot(qds','Linewidth', 3), legend('qd_1', 'qd_2')
figure, plot(ps(1,:), ps(2, :),'Linewidth', 3 ), xlabel('p_x'), ylabel('p_y')
figure, plot(errors,'Linewidth', 3 ), xlabel('time step'), ylabel('Mínimo error recta (m)')
drawrobot3d(robot, qs(:,1))
animate(robot, qs(1:2,1:15:end))


end

function qd = jacobian_control_vanilla(robot, q, v)
    J = manipulator_jacobian(robot, q);
    J = J(1:2,:);
    if abs(det(J)) < 0.05
        'debug'
    end
    qd = inv(J)*v(1:2);
end


function qd = jacobian_control_damped(robot, q, v)
J = manipulator_jacobian(robot, q);
J = J(1:2,:);
if abs(det(J)) < 0.01
        % build damped
       Jd = J+0.1*eye(2);
       [V, D] = eig(Jd)
        qd = inv(Jd)*v(1:2);
        v_real =J*qd        
else
       qd = inv(J)*v(1:2);
       %drawrobot3d(robot, q)
end
end

function qd = jacobian_control_svd(robot, q, v)
    J = manipulator_jacobian(robot, q);
    J = J(1:2,:);
    if abs(det(J)) < 0.01               
        [U,S,V] = svd(J);
        s = diag(S);
        for i=1:2
           if s(i) < 0.01
              s(i)=0.0;
           else
              s(i) = 1/s(i);
           end
        end
        iJ=V*diag(s)*U';
        % corresponding to a non-null s(i)
        u = U(:,1);
        %p = u'*v;
        %if abs(p) < 0.01
        %    v = u;
        %end
        qd = iJ*u(1:2);
    else
        qd = inv(J)*v(1:2);    
    end
end
function qd = jacobian_control_svd2(robot, q, vref)
    J = manipulator_jacobian(robot, q);
    J = J(1:2,:);
    %[u, s, v] = svd(J);
    if abs(det(J)) < 0.01               
        [U,S,V] = svd(J);
        iSc = S; % corrected S
        for i=1:2
           if S(i,i) < 0.01
              Sc(i,i)=0.0;
           else
              Sc(i,i) = 1/S(i, i);
           end
        end
        iJ=V*iSc*U';
        % corresponding to a non-null s(i)
        u = U(:,1);
        %p = u'*v;
        %if abs(p) < 0.01
        %    v = u;
        %end
        qd = iJ*u(1:2, 1);
    else
        qd = inv(J)*vref(1:2);    
    end
end


function qd = jacobian_control_svd3(robot, q, qdi, vref)
    J = manipulator_jacobian(robot, q);
    J = J(1:2,:);
    if abs(det(J)) < 0.1               
        [U,S,V] = svd(J);
        qd = V(1:2,1);
        a1 = norm(qdi-qd);
        a2 = norm(qdi+qd);
%         if a1 < a2
%             qd = -qd;
%         end
        
        %v = J*qd;       
    else
        qd = inv(J)*vref(1:2);    
    end
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


