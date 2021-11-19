% This example illustrates how to interface with Coppelia using 
% use the remote API in synchronous mode. 

% In order
% Any of the provided Coppelia scenes should 
function irb140_abb_follow_line()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 6;
    coppelia.robots{1}.end_effector.n_joints = 2;
    coppelia.dt = 50/1000; % default 50 ms
    coppelia = coppelia_start(coppelia);    
    pick_and_place(coppelia)
    coppelia_stop(coppelia);
end




function pick_and_place(coppelia)
    %load robot in arte
    global robot
    robot = load_robot('ABB', 'IRB140');
    robot_number = 1;
    % initial position
    % get current robot position in simulation
    Q0 = coppelia_get_joint_positions(coppelia, robot_number);
    
    Q1 = [-pi-pi/4 pi/4 0.1 0.2 0.3]';
    % must initialize the current robot joint positions
    robot.q = Q0;

        %Initial point
    T0 = [-1 0 0 0.3;
           0 1 0  0.3;
           0 0 -1 0.6;
           0 0 0 1];
    
    %Initial point
    T1 = [-1 0 0 0.52;
           0 1 0  0;
           0 0 -1 0.4;
           0 0 0 1];
    % Final point
    T2 = [-1 0 0 0.54;
           0 1 0  0.0;
           0 0 -1 0.35;
           0 0 0 1];
    % Final point
    T3 = [-1 0 0 0.0;
           0 1 0 -0.5;
           0 0 -1 0.8;
           0 0 0 1];
       
    open_gripper(coppelia, robot_number); 
    coppelia_wait(coppelia, 15)
    close_gripper(coppelia, robot_number);
    coppelia_wait(coppelia, 15)
    
    % Synchronous movement on joint space: specify joint values
    [qt, qdt]=AbsQPath(robot, Q0, [0 0 0 0 0 0]', 50);
    robot = move_robot(coppelia, robot, 1, qt, qdt);
    
    % Synchronous movement on joint space: specify joint values
    [qt, qdt]=AbsQPath(robot, Q1, [0 0 0 0 0 0]', 50);
    robot = move_robot(coppelia, robot, 1, qt, qdt);

    % Synchronous movement on joint space: specify T matrix
    [qt, qdt] = QPath(robot, T0, [0 0 0 0 0 0]', 40, Q0);
    robot = move_robot(coppelia, robot, 1, qt, qdt);
   
    open_gripper(coppelia, robot_number);
    coppelia_wait(coppelia, 15)

%     [qt, qdt] = LPath(robot, T1, 50);
%     robot = move_robot(coppelia, robot, 1, qt, qdt);
% 
%     [qt, qdt] = LPath(robot, T2, 50);
%     robot = move_robot(coppelia, robot, 1, qt, qdt);
%     
%     % now close gripper
%     close_gripper(coppelia, robot_number);
%     
%     [qt, qdt] = LPath(robot, T1, 20);
%     robot = move_robot(coppelia, robot, 1, qt, qdt);
% 
% 
%     [qt, qdt]=LPath(robot, T3, 20);
%     robot = move_robot(coppelia, robot, 1, qt, qdt);    
%     
%     open_gripper(coppelia, robot_number);
% 
%     [qt, qdt]=LPath(robot, T1, 20);
%     robot = move_robot(coppelia, robot, 1, qt, qdt);
% 
%     open_gripper(coppelia, robot_number);
    
end





