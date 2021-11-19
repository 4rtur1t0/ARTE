% Set joint positions in coppelia from ARTE
% q_path can be a single joint position p.e. q = [0.1 0.2 0.3 0.4 0.5 0.6]' or a path
% q_path = [q1 q2 q3...]
% In order to have a smooth and realistic movement, Joint Targets and Joint
% Speeds are set during the path. Joint paths and speeds are computed with
% a matching delta time between them.
% At the end of the function, Joint Positions are set, to ensure that the
% path is always ended at the specified joint positions
function set_joint_target_trajectory(coppelia, q_path)
    max_iter = 150;
    if (coppelia.clientID > -1)
        joint_handles = coppelia.robot.j_handles;
        % iterate along the joint path
        for j=1:size(q_path, 2)
            %j, set every q in the path
            q = q_path(:,j);
            %qd = qd_path(:, j);
            for i=1:length(joint_handles)
                %i, every joint
                coppelia.sim.simxSetJointTargetPosition(coppelia.clientID, joint_handles(i), q(i), coppelia.sim.simx_opmode_oneshot);          
                %coppelia.sim.simxSetJointTargetVelocity(coppelia.clientID, joint_handles(i), qd(i), coppelia.sim.simx_opmode_oneshot);
            end            
            %trigger the next simulation step in Coppelia
            n_iterations = 0;
            while 1 
                coppelia.sim.simxSynchronousTrigger(coppelia.clientID);
                if reached_joint_position(coppelia, q)
                    break
                end
                n_iterations = n_iterations + 1;
                if n_iterations > max_iter
                    disp('ERROR!!! COULD NOT REACH TARGET JOINT POSITION')
                    break
                end
            end
        end       
    end
   
end

function result = reached_joint_position(coppelia, q)
    q_actual = coppelia_get_joint_positions(coppelia);
    error = q(:)-q_actual(:);
    % normalize difference
    error = atan2(sin(error), cos(error));
    % norm(error)
    if norm(error) < 0.01
        result = 1;
    else
        result = 0;
    end
end

