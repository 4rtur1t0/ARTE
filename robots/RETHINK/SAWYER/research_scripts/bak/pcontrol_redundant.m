
function path_planning_redundant%(robot)
% SIMPLE CONTROL FOR SAWYER BASED ON JACOBIAN 
close all;
global robot
global path_planning_params

%actually try to reach orientation before position.
%this depends upon V = [v0 w0] --> increase angular velocity with respect
%to unitary speed velocity. Also, when end tool is near vicinity--> change
%a reduce the speed v0 to reach the final point.
Kw = 3; 


%fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds and viceversa')
%robot=load_robot('RETHINK','SAWYER');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial and end poses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%q0 = [-0.6 -1.2 0.0 0.2 0.0 1.0 0.0]';
q0 = [0.0 -0.6 0.0 1.2 0.0 1.0 0.0]';
%so, specifying qd3=0 in the solution--> caution
%qf = [0.9 -0.5 0.3 1.5 0.1 0.1 0.1]';
%qf = [-1.5 -0.65 0.5 1.9 0.1 -0.5 0.3]';
qf = [-0.8 -0.65 0.0 1.5 -1.5 -1.2 0.3]';
%total_time = 2; %seconds

%Work Piece velocity in base coordinates
Vp = [0.0 0.0 0]';

T0=directkinematic(robot, q0);
drawrobot3d(robot, q0)
drawrobot3d(robot, qf)

%initial position and orientation of the robot
Q0 = T2quaternion(T0);
P0 = T0(1:3,4);

%Elegimos el punto de destino como aquel definido por esas coordenadas
%articulares. Tenemos Q y P que definen la superficie
%Qp Pp are the initial input parameters
[Qp, Pp]=select_target_point(robot, qf);

%Now revert the transformation-> obtain --> Tf that should be issued to the
%robot.Transform to homogeneous
%obtain the transformation of the piece
TObj = quaternion2T(Qp, Pp);
%revert the coupling transformation (robot-end tool)
Tf = TObj*inv(robot.Tcoupling);

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
errorq = [];
errorpos=[];
errororient=[];
reached_pos = 0;
reached_orient = 0;
%simulate the process
while 1  
    Ti=directkinematic(robot, q);
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
    %compute linear speed and angular speed that are served as a high level
    eps1= reached_position(Pf, Pi);
    eps2= reached_orientation(Qf, Qi);
    v0 = compute_high_level_action_kinematic_v(P0, Pf, Pi); %1m/s 
    w0 = compute_high_level_action_kinematic_w(Q0, Qf, Qi); %1rad/s
    R = Ti(1:3,1:3);
    
    v0 = v0 + normrnd(0, 0.05, 1 , 3)';
    w0 = w0 + normrnd(0, 0.05, 1 , 3)';
    
    w0 = -R*w0;
    %the restriction is the speed to reach the point
    Vref = [v0' w0']';
    if eps1 < 0.01 && eps2 < 0.005
        break;
    end
    
    %So--> once the high level control action is computed--> compute the
    %low level control actions qd.     %the low level control action is:3 is the redundant joint
    qd = compute_control_action_hierarchy(robot, Vref, q, qf);
    
    %informative
    V = compute_end_velocity(robot, q, qd);
    V-Vref
    
    %actually move the robot.
    q = q + qd*step_time;
    
    Pf = Tf(1:3,4);
    %actually move the piece also
    Pf = Pf+Vp*step_time;
    Tf(1:3,4) = Pf;
    
    drawrobot3d(robot, q)
    draw_axes(Tf, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
     
    %save speed
    qqd = [qqd qd];
    %save errors
    qq = [qq q];
    errorq = [errorq q-qf];
    errorpos = [errorpos eps1];
    errororient = [errororient eps2];
end

figure, hold
plot(errorq(1, :), 'r')
plot(errorq(2, :), 'g')
plot(errorq(3, :), 'b')
plot(errorq(4, :), 'c')
plot(errorq(5, :), 'm')
plot(errorq(6, :), 'y')
plot(errorq(6, :), 'k')
legend('error q_1', 'error q_2', 'error q_3', '...')
% title('End effector position (m)')
% xlabel('time (s)')





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
