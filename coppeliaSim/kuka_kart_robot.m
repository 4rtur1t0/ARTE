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

function kuka_kart_robot()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    n_joints = 10;
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = n_joints;
    
    coppelia = coppelia_init(coppelia);
    
    q = 0.1*ones(n_joints, 1);
    dq = 0.1*q;
    % Draw robot 1: R1, which possesses joints R1_q1, R1_q2...
    for i=1:40
       q = q + dq;
       drawrobotcoppelia(coppelia, 1, q)
       pause(0.01)
    end
    
    coppelia_end(coppelia);
end
