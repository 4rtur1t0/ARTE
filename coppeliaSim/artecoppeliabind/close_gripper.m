function close_gripper(coppelia)
    %robot_name = coppelia.robot.name;
    %gripper_name = strcat(robot_name, '_gripper');
    [res retInts retFloats retStrings retBuffer]=coppelia.sim.simxCallScriptFunction(coppelia.clientID,'R1_gripper',coppelia.sim.sim_scripttype_childscript,'open_close_gripper_function',[0],[],[],[],coppelia.sim.simx_opmode_blocking);
    coppelia_wait(coppelia, 10)
end