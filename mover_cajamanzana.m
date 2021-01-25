% Funcion mover cajas manzanas %

function mover_cajamanzana(lleno_manzana)
    
    global ini_robot pos_ini_robot pos_final_box_manzana pos_intermedia_box_manzana pos_inicial_box_manzana pos_final_box_naranja pos_intermedia_box_naranja pos_inicial_box_naranja
    global robot TD_gripper 
    
    robot=load_robot('KUKA','KR30_3');
    robot.T0 = [0 1 0 0;-1 0 0 2;0 0 1 0; 0 0 0 1];
%    robot.equipment{1}=load_robot('proyecto','cinta_apple');
%    robot.equipment{2}=load_robot('proyecto','cinta_orange');
   % robot.tool= load_robot('proyecto','gripper_1');
   % robot.equipment{3}=load_robot('proyecto','fruit_box_orange');
    %robot.equipment{3}.T0=[0 0 1 -0.5; -1 0 0 0.2; 0 1 0 0.6; 0 0 0 1]
    %robot.equipment{4}=load_robot('proyecto','fruit_box_apple');
    %robot.equipment{4}.T0=[0 0 1 0.5; -1 0 0 0.2; 0 1 0 0.6; 0 0 0 1]
    %robot.equipment{5}=load_robot('proyecto','fruit_box_apple');
    %robot.equipment{5}.T0=[1 0 0 -1.25;0 0 1 1; 0 1 0 1.26; 0 0 0 1]
    %robot.equipment{6}=load_robot('proyecto','fruit_box_apple');
    %robot.equipment{6}.T0=[1 0 0 1.25;0 0 1 1; 0 1 0 1.26; 0 0 0 1]
    
    TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];
    %Posiciones Robot cajas
    ini_robot=[0,0,0,0,0,0]
    pos_ini_robot=[[0.0000,0.5350,1.8100],[0.5000,0.5000,-0.5000,0.5000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    pos_inicial_box_manzana=[[-0.4973,0.3205,0.6191],[0.5166,0.5500,-0.4840,0.4432],[-1,-1,0,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
    pos_intermedia_box_manzana=[[-0.5953,0.6395,1.2736],[0.5927,0.4117,-0.5790,0.3795],[-1,-1,0,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
    pos_final_box_manzana=[[-1.1121,0.9816,1.3237],[0.7567,0.0412,-0.6445,0.1018],[-1,0,-1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
    MoveAbsJ(ini_robot,'v500','fine',TD_gripper, 'wobj0');
    %1
    %MoveAbsJ(pos_inicial_box_manzana,'v500','fine',TD_gripper, 'wobj0');
    %2
    MoveJ(pos_inicial_box_manzana,'vmax','z100',TD_gripper, 'wobj0');
    MoveAbsJ(pos_intermedia_box_manzana,'v500','fine',TD_gripper, 'wobj0');
    %MoveJ(pos_intermedia_box_manzana,'v500','fine',TD_gripper, 'wobj0');
    MoveAbsJ(pos_final_box_manzana,'v500','fine',TD_gripper, 'wobj0');
    %MoveJ(pos_final_box_manzana,'v500','fine',TD_gripper, 'wobj0');
    
    MoveAbsJ(ini_robot,'v500','fine',TD_gripper, 'wobj0');
    
    drawrobot3d(robot)
end
