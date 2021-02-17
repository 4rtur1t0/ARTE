% Set joint positions in coppelia from ARTE
% q_path can be a single joint position p.e. q = [0.1 0.2 0.3 0.4 0.5 0.6]' or a path
% q_path = [q1 q2 q3...]
function move_robot(coppelia, robot_index, q_path, qd_path)
    if (coppelia.clientID > -1)
        joint_handles = coppelia.robots{robot_index}.j_handles;
        % iterate along the joint path
        for j=1:size(q_path, 2)
            %j, set every q in the path
            q = q_path(:,j);
            qd = qd_path(:, j);
            for i=1:length(joint_handles)
                %i, every joint
                %coppelia.sim.simxSetJointPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);     
                coppelia.sim.simxSetJointTargetPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);          
                coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, joint_handles(i), qd(i), coppelia.sim.simx_opmode_oneshot);
            end
            coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
        end       
    end
    coppelia_wait(coppelia, 5)    
end
