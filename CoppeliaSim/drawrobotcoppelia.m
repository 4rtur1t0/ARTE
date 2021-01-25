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

function drawrobotcoppelia(simulation, q_path)
    %global simulation
    
    if (simulation.clientID > -1)
        for j=1:size(q_path,2)
            %j
            q = q_path(:,j);
            for i=1:simulation.n_joints
              simulation.sim.simxSetJointPosition(simulation.clientID, simulation.joint_handles(i), q(i), simulation.sim.simx_opmode_oneshot);
            end
            simulation.sim.simxSynchronousTrigger(simulation.clientID);
        end       
    end
end
