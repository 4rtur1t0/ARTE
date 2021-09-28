
function open_gripper(coppelia, robot_number)
    robot_name = strcat('R', int2str(robot_number));
    gripper_name = strcat(robot_name, '_gripper');
    
    if gripper_name == 'R2_gripper'
        gripper_name='R1_gripper#0';
    end
    
    [res retInts retFloats retStrings retBuffer]=coppelia.sim.simxCallScriptFunction(coppelia.clientID,gripper_name,coppelia.sim.sim_scripttype_childscript,'open_close_gripper_function',[1],[],[],[],coppelia.sim.simx_opmode_blocking);
    %coppelia_wait(coppelia, 10)
end
