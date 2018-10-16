
%   INVERSE KINEMATICS FOR THE SAWYER ROBOT
%
%   Solves the inverse kinematic problem in various situations.
%   A Jacobian based method is used.
%   The method tries to reach the given position/orientation while, at the 
%   same time, maximizing/minimizing a secondary target.
%
%   e.g. reach position/orientation and maximize manipulability det(J'J)
%   
%   Mode 0: 
%       Return the solution
function q = inverse_kinematics_sawyer%(robot)
close all;
global robot


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%q0 = [0.0 -0.6 0.0 1.2 0.0 1.0 0.0]';
q0 = [0.0 -0.4 0.1 1.0 0.1 0.1 0.1]';
qf = [0.0 -0.8 0.2 1.5 0.5 1.0 0.5]';

T0=directkinematic(robot, q0);
drawrobot3d(robot, q0)
drawrobot3d(robot, qf)

Tf=directkinematic(robot, qf);

%initial position and orientation of the robot
Q0 = T2quaternion(T0);
P0 = T0(1:3,4);
%puntos finales de la planificación
%orientación en el sistema de coordenadas de la base
Qf = T2quaternion(Tf);
Pf = Tf(1:3,4);

close all
drawrobot3d(robot, q0)
draw_axes(Tf, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);

q=q0;
step_time = 0.05;
qq = [];
qqd = [];
manips = [];
%step 1, inverse kinematics
%this is a gradient descent solution based on moore-penrose inverse
while 1  
    J = manipulator_jacobian(robot, q);
    manip = det(J*J');
    manips = [manips manip];
    
    Ti=directkinematic(robot, q);
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
   
    eps1= reached_position(Pf, Pi);
    eps2= reached_orientation(Qf, Qi);
    %compute linear speed and angular speed that are served as a high level
    %based on the current pose
    v0 = compute_high_level_action_kinematic_v(P0, Pf, Pi); %1m/s 
    w0 = compute_high_level_action_kinematic_w(Q0, Qf, Qi); %1rad/s
    R = Ti(1:3,1:3);
    w0 = -R*w0;
    %the restriction is the speed to reach the point
    Vref = [v0' w0']';
    if eps1 < 0.01 && eps2 < 0.005
        break;
    end
    
    %So--> once the high level control action is computed--> compute the
    %low level control actions qd.     %the low level control action is:3 is the redundant joint
    qd = compute_control_action_hierarchy(robot, Vref, q, 'q3_0');
   
    %actually move the robot.
    q = q + qd*step_time;
    
    drawrobot3d(robot, q)
    
    %save speed
    qqd = [qqd qd];
    %save errors
    qq = [qq q];
end

drawrobot3d(robot, q)
draw_axes(Tf, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);

figure, hold
plot(manips, 'r')
legend('manipulability')




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
