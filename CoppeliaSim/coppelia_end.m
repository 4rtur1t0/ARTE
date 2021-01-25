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

function coppelia_end(simulation)
    %global simulation
    if (simulation.clientID>-1)
        disp('Already connected to remote API server: trying to stop simulation');
        % stop the simulation:
        simulation.sim.simxStopSimulation(simulation.clientID,simulation.sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        simulation.sim.simxFinish(simulation.clientID);
    else
        disp('Failed connecting to remote API server to stop simulation');
    end
    simulation.sim.delete();
end
