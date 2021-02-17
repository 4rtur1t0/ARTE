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

function two_kuka_robots()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 7;
    coppelia.robots{1}.end_effector.n_joints = 0;
    coppelia.robots{2}.n_joints = 7;
    coppelia.robots{2}.end_effector.n_joints = 0;

    coppelia = coppelia_init(coppelia);
    
    q = [0.1 0.2 0.3 0.4 0.5 0.6 0.7]';
    dq = 0.1*[1 1 1 1 1 1 1]';
    % Draw robot 1: R1, which possesses joints R1_q1, R1_q2...
    for i=1:140
       q = q + dq;
       drawrobotcoppelia(coppelia, 1, q)
       drawrobotcoppelia(coppelia, 2, q)
       pause(0.1);
    end
    
    coppelia_end(coppelia);
end
