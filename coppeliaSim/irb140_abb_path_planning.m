% This small example illustrates how to use the remote API
% synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:
%
% simRemoteApi.start(19999,1300,false,true)
%
% But in this example we try to connect on port
% 19997 where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function irb140_abb_path_planning()
    global configuration
  
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 6;
    coppelia.robots{1}.end_effector.n_joints = 2;
    coppelia.dt = 50/1000; % default 50 ms
    %should match ARTE delta_time
    configuration.delta_time = 50/1000;
    %configuration.delta_time = 0.01;
    %coppelia = coppelia_start(coppelia);
    
    pick_and_place(coppelia)
    %coppelia_stop(coppelia);
end




function pick_and_place(coppelia)
%load in arte
global robot
robot = load_robot('ABB', 'IRB140');
robot_number = 1;
% initial position
Q = [0 0 0 0 0 0]';
robot.q = Q;
robot.qd = [0 0 0 0 0 0]';
lin_vel = 0.5; % m/s
ang_vel = 0.8; % rad/s

% %Initial point
% T1 = [-1 0 0 0.52;
%        0 1 0  0;
%        0 0 -1 0.4;
%        0 0 0 1];
% % Final point
% T2 = [1 0 0 0.5;
%        0 1 0  0.5;
%        0 0 1 0.5;
%        0 0 0 1];
% % Final point
% T3 = [-1 0 0 0.5;
%        0 1 0  -0.5;
%        0 0 -1 0.5;
%        0 0 0 1];
q2 = [0.5 0.5 0.5 0.5 0.5 0.5];
qd2 = [0 0 0 0 0 0];
% end speed is 20 % of max speed for all joints   
[q, qdd]=AbsJPath(robot, q2, qd2, 20);
move_robot(coppelia, 1, q, qdd)



% go to pre pick point, then open, then to pick point
%MoveQ(coppelia, robot_number, T1)
%open_gripper(coppelia, robot_number);
%coppelia_wait(coppelia, 10)
%MoveL(coppelia, robot_number, T2, lin_vel, ang_vel)
%MoveL(coppelia, robot_number, T3, lin_vel, ang_vel)
%MoveL(coppelia, robot_number, T1, lin_vel, ang_vel)

% % now close gripper
% close_gripper(coppelia, robot_number);
% MoveQ(coppelia, robot_number, T1)
% 
% 
% %release
% MoveQ(coppelia, robot_number, T3)
% open_gripper(coppelia, robot_number);
% % over the release point
% MoveQ(coppelia, robot_number, T4)
% %start point again
% MoveAbsQ(coppelia, robot_number, Q)
end


% % Linear interpolation move to Q
% function MoveAbsQ(coppelia, robot_number, Q)
%     global robot
%     q0 = robot.q;
%     %q1 = [0.0 0.0 0.0 0.0 0.0 0.0]';
%     q_path = linear_q_path(q0, Q);
%     setjointtargetpositions(coppelia,robot_number, q_path);    
%     %coppelia_wait(coppelia, 10)
%     robot.q = Q;
% end

% % MoveL
% % MoveQ
% function MoveQ(coppelia, robot_number, T)
% %drawrobot3d(robot, q0)
%     global robot
%     %start joint coordinates, as saved before
%     q0 = robot.q;
%     
%     qinv = inversekinematic(robot, T);
%     q1 = qinv(:,1);
%     q_path = linear_q_path(q0, q1);
%     setjointtargetpositions(coppelia, robot_number, q_path)
%     %coppelia_wait(coppelia, 10)
%     robot.q = q1;
% end

function MoveL(coppelia, robot_number, Tf, lin_vel, ang_vel)

    global robot
    %start joint coordinates, as saved before
    q0 = robot.q;
    T0 = directkinematic(robot, q0);
    p0 = T0(1:3, 4);
    pf = Tf(1:3, 4);
    
    qinv = inversekinematic(robot, Tf);
    qf = qinv(:,1);
    Q0 = T2quaternion(T0);
    Qf = T2quaternion(Tf);
    
    v0 = abs(lin_vel)*compute_speed(p0, pf);
    w0 = abs(ang_vel)*compute_ang_speed(Q0, Qf);
    Vref = [v0' w0']';
    qs = [robot.q];
    qds = [robot.qd];
    q = q0;
    while 1
        Ti = directkinematic(robot, q);
        pi = Ti(1:3, 4);
        e = norm(pf - pi);
        if e < 0.05
            break
        end
        J = manipulator_jacobian(robot, q);
        qd = inv(J)*Vref;
        q = q + qd*coppelia.dt;
        qs = [qs q];
        qds = [qds qd];        
    end
    %q_path = linear_q_path(q0, q1);
    move_robot(coppelia, robot_number, qs, qds);
    %coppelia_wait(coppelia, 10)
    robot.q = q;
    robot.qd = qd;
end



function q_path = linear_q_path(q0, q1)
q_path=[];
% find max difference
dq = abs(q0-q1);
[y, i] = max(dq);
n = floor(y/0.1 + 1);
for i=1:length(q0)
    q_path = [q_path; linspace(q0(i), q1(i), n)];
end
%q_path
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_speed(Pi, Pf)
%compute a constant linear speed till target
v = (Pf-Pi);
v = v(:)/norm(v);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [w] = compute_ang_speed(Qi, Qf)
%compute a constant angular speed till target
%asume the movement is performed in 1 second
w = angular_w_between_quaternions(Qi, Qf, 1);
w = w(:);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute angular speed w that moves Q0 into Q1 in time total_time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = angular_w_between_quaternions(Q0, Q1, total_time)
%global robot
%below this number, the axis is considered as [1 0 0]
%this is to avoid numerical errors
%this is the actual error allowed for w
%epsilon_len = robot.parameters.epsilonQ;
epsilon_len = 0.0001;
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);

if len > epsilon_len
    angle=2*atan2(len, Q(1));
    axis=[Q(2) Q(3) Q(4)]./len;
else
    angle=0;
    axis=[1 0 0];
end
w=axis*angle/total_time;
end


