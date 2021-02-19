% This example illustrates how to interface with Coppelia using 
% use the remote API in synchronous mode. 
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
    Q0 = [0 0 0 0 0 0]';
    Q1 = [pi/4 pi/4 -pi/4 0.1 0.2 0.3];
    Q2 = [0.3 0.3 0.3 0.3 0.3 0.3];
    Q3 = [0.15 0.15 0.15 0.15 0.15 0.15];
    Q4 = [0.1 0.1 0.1 0.1 0.1 0.1]
    % must initialize the current robot joint positions
    robot.q = Q0;

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
    [qt, qdt]=AbsJPath(robot, Q3, [0 0 0 0 0 0], 30);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);

    %drawrobot3d(robot, robot.q)
    [qt, qdt] = MoveLPath(robot, T1, 20);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);
    [qt, qdt] = MoveLPath(robot, T2, 50);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);

    % % now close gripper
    close_gripper(coppelia, robot_number);
    [qt, qdt] = MoveLPath(robot, T1, 20);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);

    [qt, qdt]=MoveLPath(robot, T3, 20);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);
    open_gripper(coppelia, robot_number);
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);

    [qt, qdt]=MoveLPath(robot, T1, 20);
    move_robot(coppelia, 1, qt, qdt)
    robot.q = qt(:,end);
    robot.qd=qdt(:,end);



end





