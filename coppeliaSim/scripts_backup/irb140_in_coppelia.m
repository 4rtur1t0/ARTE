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

function irb140_in_coppelia()
    % init basic data. Robots and number of collision objects.
    coppelia = [];
    coppelia.n_collision_objects = 0;
    coppelia.robots{1}.n_joints = 6;
    coppelia.robots{1}.end_effector.n_joints = 0;
    
    coppelia = coppelia_init(coppelia);
    
    q = [0.1 0.2 0.3 0.4 0.5 0.6]';
    dq = 0.01*[1 1 1 1 1 1]';
    % Draw robot 1: R1, which possesses joints R1_q1, R1_q2...
    for i=1:50
       q = q + dq;
       drawrobotcoppelia(coppelia, 1, q)
       %gripper(coppelia, 1, 1);
       %coppelia.sim.simxSynchronousTrigger(coppelia.clientID); 
       %pause(0.001)
    end
    
    coppelia_end(coppelia);
end

function gripper(coppelia, robot_number, open_close)

j_handles = coppelia.robots{robot_number}.end_effector.j_handles;
                
[r, p1]=coppelia.sim.simxGetJointPosition(coppelia.clientID, j_handles(1), coppelia.sim.simx_opmode_oneshot);
[r, p2]=coppelia.sim.simxGetJointPosition(coppelia.clientID, j_handles(1), coppelia.sim.simx_opmode_oneshot);

if open_close==1
    coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, j_handles(1), -0.02, coppelia.sim.simx_opmode_oneshot);
    coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, j_handles(2), -0.02, coppelia.sim.simx_opmode_oneshot);
else
    coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, j_handles(1), 0.02, coppelia.sim.simx_opmode_oneshot);
    coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, j_handles(2), 0.02, coppelia.sim.simx_opmode_oneshot);
end

end