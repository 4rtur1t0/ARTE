% Set joint positions in coppelia from ARTE
% q_path can be a single joint position p.e. q = [0.1 0.2 0.3 0.4 0.5 0.6]' or a path
% q_path = [q1 q2 q3...]
% In order to have a smooth and realistic movement, Joint Targets and Joint
% Speeds are set during the path. Joint paths and speeds are computed with
% a matching delta time between them.
% At the end of the function, Joint Positions are set, to ensure that the
% path is always ended at the specified joint positions
function robot = move_robot(coppelia, robot_index, q_path, qd_path)
    if (coppelia.clientID > -1)
        joint_handles = coppelia.robots{robot_index}.j_handles;
        % iterate along the joint path
        for j=1:size(q_path, 2)
            %j, set every q in the path
            q = q_path(:,j);
            qd = qd_path(:, j);
            for i=1:length(joint_handles)
                %i, every joint
                coppelia.sim.simxSetJointTargetPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);          
                coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, joint_handles(i), qd(i), coppelia.sim.simx_opmode_oneshot);
            end
            %trigger the next simulation step in Coppelia
            coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
        end       
    end
    % Set last joint positions
   % for i=1:length(joint_handles)
   %     coppelia.sim.simxSetJointPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);
   % end
    count = 0;
    max_iter = 50;
    while count < max_iter            
        coppelia.sim.simxSynchronousTrigger(coppelia.clientID);

        %coppelia_wait(coppelia, 1)
        q_actual = coppelia_get_joint_positions(coppelia, robot_index);
        error = q(:)-q_actual(:);
        norm(error)
        if norm(error) < 0.01
            break
        end
        for i=1:length(joint_handles)
             coppelia.sim.simxSetJointPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);
        end
        count = count + 1;
    end
    
    if count==max_iter
        disp('ERROR!!! COULD NOT REACH TARGET JOINT POSITION')
    end
    
    % this waits 5 steps in simulation
    %coppelia_wait(coppelia, 5)    
    %q_actual = coppeliagetjointpositions(coppelia, robot_index);
    %error = q(:)-q_actual(:);
    robot.q = q(:);
    robot.qd = qd(:);
end

% function q_actual = coppeliagetjointpositions(coppelia, robot_index)
%     joint_handles = coppelia.robots{robot_index}.j_handles;
% 
%     q_actual = [];
%     for i=1:length(joint_handles)
%         [error, value]=coppelia.sim.simxGetJointPosition(coppelia.clientID, joint_handles(i), coppelia.sim.simx_opmode_buffer);
%         q_actual = [q_actual value];
%     end
% end
