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

function coppelia = coppelia_start(coppelia)
    disp('Program started: TRYING TO COMMUNICATE WITH COPPELIA');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    coppelia.sim = sim;
    coppelia.clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (coppelia.clientID>-1)
        disp('Connected to remote API server');
        % enable the synchronous mode on the client:
        sim.simxSynchronous(coppelia.clientID,true);

        % start the simulation:
        sim.simxStartSimulation(coppelia.clientID, sim.simx_opmode_blocking);
        coppelia = retrieve_collision_handles(coppelia);
        coppelia = retrieve_robot_handles(coppelia);
        %coppelia = retrieve_end_effector_handles(coppelia);
      
    else
        disp('Failed connecting to remote API server');
    end
end


function coppelia = retrieve_collision_handles(coppelia)
    n_collision_objects = coppelia.n_collision_objects;
    c_handles = [];
    for i=1:n_collision_objects
        collision_obj_name = strcat('Collision');
        [r, c_handles(i)]=sim.simxGetObjectHandle(coppelia.clientID, collision_obj_name, coppelia.sim.simx_opmode_blocking);
    end
    coppelia.collisions.c_handles = c_handles;    
end



function coppelia = retrieve_robot_handles(coppelia)

    n_joints = coppelia.robot.n_joints;
    j_handles = [];
    for i=1:n_joints
        robot_name = coppelia.robot.name;
        joint_name = strcat('_q', int2str(i));
        full_joint_name = strcat(robot_name, joint_name);
        [r, j_handles(i)]=coppelia.sim.simxGetObjectHandle(coppelia.clientID, full_joint_name, coppelia.sim.simx_opmode_blocking);
    end
    coppelia.robot.j_handles = j_handles;
        
    n_joints = coppelia.robot.end_effector.n_joints;
    j_handles=[];
    for i=1:n_joints
        robot_name = coppelia.robot.name;
        joint_name = strcat('_gripper_q', int2str(i));
        full_joint_name = strcat(robot_name, joint_name);
        [r, j_handles(i)]=coppelia.sim.simxGetObjectHandle(coppelia.clientID, full_joint_name, coppelia.sim.simx_opmode_blocking);
    end
        coppelia.robot.end_effector.j_handles = j_handles;
    

    q_actual = [];
    joint_handles = coppelia.robot.j_handles;
    for i=1:length(joint_handles)
        [error, value]=coppelia.sim.simxGetJointPosition(coppelia.clientID, joint_handles(i), coppelia.sim.simx_opmode_streaming);
        q_actual = [q_actual value];        
    end
    
end

% 
% function coppelia = retrieve_end_effectors_handle(coppelia)
% 
%     n_robots = size(coppelia.robots, 2);
%     for j=1:n_robots
%         n_robot_hands = coppelia.robots{j}..has_end_effector;
%         if has_end_effector
%             robot_name = strcat('R', int2str(j));
%             hand_name = '_effector';
%             full_joint_name = strcat(robot_name, joint_name);
%             [r, end_effector_handle]=coppelia.sim.simxGetObjectHandle(coppelia.clientID, full_joint_name, coppelia.sim.simx_opmode_blocking);
%         end
%         coppelia.robots{j}.end_effector_handle = end_effector_handle;
%     end
% end
    