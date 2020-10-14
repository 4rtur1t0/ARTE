function proyecto_coche
close all;
global robot

robot1=load_robot('ABB','IRB1410/right');
robot2=load_robot('ABB','IRB1410/left');
cd ..

robot=[];
robot.name='IRB1410';
robot.robot1=robot1;
robot.robot2=robot2;
robot.nserial=2;

robot.robot1.graphical.draw_axes=0;
robot.robot1.graphical.has_graphics = 1;
robot.robot2.graphical.draw_axes=0;
robot.robot2.graphical.has_graphics = 1;

adjust_view(robot.robot1)
robot.robot1.equipment{1}=load_robot('equipment', 'end_tools/pallet_gripper');
robot.robot1.equipment{1}.graphical.draw_axes=0;
robot.robot1.equipment{1}.T0(1:3,4) = [-5;0.4;4.35];

robot.robot1.piece{1}= load_robot('equipment', 'bodywork/aston_martin');   
robot.robot1.piece{1}.T0(1:3,4) = [-1.75;0;0.8]';
robot.robot1.piece{1}.graphical.draw_axes=0;
robot.robot1.piece{1}.graphical.has_graphics = 1;

 %   C) NOW, LOAD AN END TOOL
robot.robot1.tool= load_robot('equipment','end_tools/vacuum_1');
robot.robot2.tool= load_robot('equipment','end_tools/vacuum_1');

cd ../../..
cd ABB/IRB1410

robot.robot1.T0(1:3,4)=[-7 1.5 1.5]';  % con esto defino la posicion del robot 1
robot.robot2.T0(1:3,4)=[-7 -1.5 1.5]'; % con esto defino la posicion del robot 2 

q0 = [0   0  0    0    0    0 ]';
q1 = [-1.008   0.6963   0.74    -1.002   -1.445    -1.4660 ]'; 
%q1 = [0   1.0262   0.3787    0.05   -1.4049    1.5707 ]'; 


step_time = 0.05;
qdi = 0;
t=[0 2];
[q1_t, qd1_t, qdd1_t, time, k]=second_order([q0(1) q1(1)], qdi,[t(1) t(2)], step_time);
[q2_t, qd2_t, qdd2_t, time, k]=second_order([q0(2) q1(2)], qdi,[t(1) t(2)], step_time);
[q3_t, qd3_t, qdd3_t, time, k]=second_order([q0(3) q1(3)], qdi,[t(1) t(2)], step_time);
[q4_t, qd4_t, qdd4_t, time, k]=second_order([q0(4) q1(4)], qdi,[t(1) t(2)], step_time);
[q5_t, qd5_t, qdd5_t, time, k]=second_order([q0(5) q1(5)], qdi,[t(1) t(2)], step_time);
[q6_t, qd6_t, qdd6_t, time, k]=second_order([q0(6) q1(6)], qdi,[t(1) t(2)], step_time);
i=1;

q=q1;
Ti= directkinematic(robot.robot1, q);
    Pi = Ti(1:3,4)
    
    robot.robot1.piece{1}.T0(1:3,4) = Pi;
    robot.robot1.piece{1}.T0(1,4) = robot.robot1.piece{1}.T0(1,4) +1.75;
    robot.robot1.piece{1}.T(2,4) =  robot.robot1.piece{1}.T0(2,4)-3;
    robot.robot1.piece{1}.T(3,4) =  robot.robot1.piece{1}.T0(3,4);
 
for n = time   
    qt = [q1_t(i) q2_t(i) q3_t(i) q4_t(i) q5_t(i) q6_t(i) ]'; 
    i=i+1;
    qti = brazos(qt); 
    dibujar_robot1410(robot.robot1, qt, robot.robot2, qti);
end

t = 0;
q_p = [q];
time = [t];
while t <= 2
    J = manipulator_jacobian(robot.robot1, q);
    
    Ti= directkinematic(robot.robot1, q);
    Pi = Ti(1:3,4); 
    
    robot.robot1.piece{1}.T0(1:3,4) = Pi;
    robot.robot1.piece{1}.T0(1,4) = robot.robot1.piece{1}.T0(1,4) +1.25;
    robot.robot1.piece{1}.T(2,4) =  robot.robot1.piece{1}.T0(2,4)-3;
    robot.robot1.piece{1}.T(3,4) =  robot.robot1.piece{1}.T0(3,4);
   
    vx = 0.07*t^2 -0.2*t + 0.284;
    vy = 0.4*t^2 -0.1*t + 0.384; 
    vz = -0.4*t^2 + 1.3*t - 0.3;
    v = [vx vy vz 0 0 0]';
    
    qd = compute_control_action_hierarchyABB_1410(robot.robot1, v, q, '');
    qizq =  brazos(q);
    
    dibujar_robot1410(robot.robot1, q, robot.robot2, qizq)
    q =q+qd*step_time;
    t =t+step_time;
    q_p=[q_p q];
    time=[time t];
    
end


figure, xlabel('t (s)'), ylabel('q (rad)'), title('trayectoria realizada por las articulaciones'), hold on
plot(time, q_p, 'b')

function [q_t, qd_t, qdd_t, time, k]=second_order(q, qd, t, delta_t)
k=inv([1 t(1) t(1)^2; 1 t(2) t(2)^2; 0 1 2*t(1)])*[q(1) q(2) qd(1)]';
time=t(1):delta_t:t(2);
q_t=k(1)+k(2)*time+k(3)*time.^2;
qd_t=k(2)*ones(1,length(time)) + 2*k(3)*time;
qdd_t=2*k(3)*ones(1,length(time));