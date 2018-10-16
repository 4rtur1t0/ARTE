
%   MAXIMIZE manipulability along the null space
% use a simple delta manip to stop.
% whenever the increase in manipulability is below a threshold, stop
%
%  USES A local technique
function q = optimize_manip_local(robot, q0, objective, delta)
%T0=directkinematic(robot, q0);
q=q0;
step_time = 0.1;
manips = [];
delta_manips = [];
%step 1, inverse kinematics
%this is a gradient descent solution based on moore-penrose inverse
while 1  
    J = manipulator_jacobian(robot, q);
    manip0 = det(J*J');
    manips = [manips manip0];
    %the restriction is the speed is zero to linear and angular speeds
    % move the redundant joints in order to maximize manipulability
    Vref = [0 0 0 0 0 0]';
    %So--> once the high level control action is computed--> compute the
    %low level control actions qd.     %the low level control action is:3 is the redundant joint
    [qd] = compute_control_action_hierarchy(robot, Vref, q, objective);
    %actually move the robot.
    q = q + qd*step_time;
    
    J = manipulator_jacobian(robot, q);
    manip1 = det(J*J');
    %compute a delta in manip taking into account the true movement of
    %delta_Q
    delta_manip_loop = abs(manip1-manip0)/step_time;
    delta_manips = [delta_manips delta_manip_loop];
    %just a silly stop condition
    %the manipulability is not, at all, global
    %delta_manip_loop
    if abs(delta_manip_loop) < delta
        break
    end
    drawrobot3d(robot, q)
end
 
% figure, hold
% plot(manips, 'r')
% legend('manipulability')
% 
% figure, hold
% plot(delta_manips, 'b')
% legend('delta_manipulability')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_high_level_action_kinematic_v(P0, Pf, Pi, vreach)
%compute a constant linear speed till target
v = (Pf-Pi);
%v = normalize(v);
%dist = sqrt(sum((Pf-Pi).^2));

%dist = saturate(dist, 0.1, 3);
%v = dist*v;
% K=3;
% %speed law with distance. Maximum speed between distance and vreach
% modv=min([3*dist vreach]);
% v = modv*v;
v = v(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [w] = compute_high_level_action_kinematic_w(Q0, Qf, Qi)
%compute a constant angular speed till target
%asume the movement is performed in 1 second
w = angular_w_between_quaternions(Qi, Qf, 1);
%dist = sqrt(sum((Qf-Qi).^2));
%dist = saturate(dist, 0.1, 1);
%w = dist*w;
%w = normalize(w);
w = w(:);


function k = saturate(k, min_value, max_value)
if k < min_value
    k = min_value;
end
if k > max_value
    k = max_value;
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute angular speed w that moves Q0 into Q1 in time total_time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = angular_w_between_quaternions(Q0, Q1, total_time)
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);
angle=2*atan2(len, Q(1));
%vector3 axis;
if len > 0
    axis=[Q(2) Q(3) Q(4)]./len;
else
    axis=[1 0 0];
end
w=axis*angle/total_time;


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
