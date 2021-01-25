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

function collision_detected=vrep_check_collision(q_path)
global simulation
clientID=simulation.clientID;
sim=simulation.sim;
j_handles=simulation.j_handles;
c_handles=simulation.c_handles;

collisions = [];
for j=1:size(q_path,2)
      q = q_path(:,j);
      sim.simxPauseCommunication(clientID,1);
      for i=1:length(j_handles)
           % sim.simxSetJointPosition(clientID, j_handles(i), q(i), sim.simx_opmode_blocking);
           sim.simxSetJointPosition(clientID, j_handles(i), q(i), sim.simx_opmode_oneshot);
      end
      sim.simxPauseCommunication(clientID,0);
      
      sim.simxSynchronousTrigger(clientID);
       
      for i=1:length(c_handles)
           [exec_ok, collisions(i)] = sim.simxReadCollision(clientID, c_handles(i), sim.simx_opmode_blocking);
           %exec_ok
      end
end

if sum(collisions > 0)
    collision_detected = 1;
else
    collision_detected = 0;
end
