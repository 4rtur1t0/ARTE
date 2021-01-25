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

function vrep_exit(sim)
% stop the simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
% Now close the connection to CoppeliaSim:    
sim.simxFinish(clientID);
sim.delete(); % call the destructor!
    
disp('Disconnecting from Coppelia simulator');

