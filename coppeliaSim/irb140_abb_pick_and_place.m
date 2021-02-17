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

function irb140_abb_pick_and_place()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 6;
    coppelia.robots{1}.end_effector.n_joints = 2;
    
    coppelia = coppelia_start(coppelia);
    
    pick_and_place(coppelia)
    % robot_reset(coppelia)   
    coppelia_stop(coppelia);
end




function pick_and_place(coppelia)
%load in arte
global robot
robot = load_robot('ABB', 'IRB140');
robot_number = 1;
% initial position
Q = [0 0 0 0 0 0]';
robot.q = Q;

%T1, pre pick point
T1 = [-1 0 0 0.52;
       0 1 0  0;
       0 0 -1 0.4;
       0 0 0 1];
% pick point
T2 = [-1 0 0 0.52;
       0 1 0  0;
       0 0 -1 0.23;
       0 0 0 1];
% Release point
T3 = [-1 0 0 0.1;
     0 1 0  -0.5;
     0 0 -1 0.23;
     0 0 0 1]; 
T4 = [-1 0 0 0.1;
     0 1 0  -0.5;
     0 0 -1 0.4;
     0 0 0 1]; 
MoveAbsQ(coppelia, robot_number, Q)
    coppelia_wait(coppelia, 10)

% go to pre pick point, then open, then to pick point
MoveQ(coppelia, robot_number, T1)
open_gripper(coppelia, robot_number); 
coppelia_wait(coppelia, 10)
MoveQ(coppelia, robot_number, T2)
coppelia_wait(coppelia, 10)
% now close gripper
close_gripper(coppelia, robot_number);
MoveQ(coppelia, robot_number, T1)


%release
MoveQ(coppelia, robot_number, T3)
open_gripper(coppelia, robot_number);
% over the release point
MoveQ(coppelia, robot_number, T4)
%start point again
MoveAbsQ(coppelia, robot_number, Q)
end


% Linear interpolation move to Q
function MoveAbsQ(coppelia, robot_number, Q)
    global robot
    q0 = robot.q;
    %q1 = [0.0 0.0 0.0 0.0 0.0 0.0]';
    q_path = linear_q_path(q0, Q);
    setjointtargetpositions(coppelia,robot_number, q_path);    
    %coppelia_wait(coppelia, 10)
    robot.q = Q;
end

% MoveL
% MoveQ
function MoveQ(coppelia, robot_number, T)
%drawrobot3d(robot, q0)
    global robot
    %start joint coordinates, as saved before
    q0 = robot.q;
    
    qinv = inversekinematic(robot, T);
    q1 = qinv(:,1);
    q_path = linear_q_path(q0, q1);
    setjointtargetpositions(coppelia, robot_number, q_path)
    %coppelia_wait(coppelia, 10)
    robot.q = q1;
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


