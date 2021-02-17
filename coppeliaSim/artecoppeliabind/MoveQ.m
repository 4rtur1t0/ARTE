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