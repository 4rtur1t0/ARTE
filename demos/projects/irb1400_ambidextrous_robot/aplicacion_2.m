% Authors: Marc Fabregat y Antonio Martinez
% Universidad Miguel Hernández de Elche
% Enero de 2020
function aplicacion_2
close all;
global robot

robot1=load_robot('ABB','IRB14000/right');
robot2=load_robot('ABB','IRB14000/left');
cd ..

robot=[];
robot.name='IRB14000';
robot.robot1=robot1;
robot.robot2=robot2;
robot.nserial=2;

adjust_view(robot.robot1)
robot.robot1.equipment{1}=load_robot('equipment', 'objects/fruit_box');
robot.robot1.equipment{1}.graphical.draw_axes=0;
robot.robot1.equipment{2}=load_robot('equipment', 'tables/table_small');
robot.robot1.equipment{2}.T0(1:3,4) = [0.57;-0.09;0.22];
robot.robot1.equipment{2}.graphical.draw_axes=0;
robot.robot1.equipment{2}.graphical.has_graphics = 0;
%cd ..\..\..
%cd ABB/IRB14000


q0 = [0 0 0 0 0 0 0]';
q1 = [1.1343   -2.3283   -1.8404    0.6438   -1.3120    1.1148   -0.5362]';

step_time = 0.05;


qdi = 0;
t=[0 2];
[q1_t, qd1_t, qdd1_t, time, k]=second_order([q0(1) q1(1)], qdi,[t(1) t(2)], step_time);
[q2_t, qd2_t, qdd2_t, time, k]=second_order([q0(2) q1(2)], qdi,[t(1) t(2)], step_time);
[q3_t, qd3_t, qdd3_t, time, k]=second_order([q0(3) q1(3)], qdi,[t(1) t(2)], step_time);
[q4_t, qd4_t, qdd4_t, time, k]=second_order([q0(4) q1(4)], qdi,[t(1) t(2)], step_time);
[q5_t, qd5_t, qdd5_t, time, k]=second_order([q0(5) q1(5)], qdi,[t(1) t(2)], step_time);
[q6_t, qd6_t, qdd6_t, time, k]=second_order([q0(6) q1(6)], qdi,[t(1) t(2)], step_time);
[q7_t, qd7_t, qdd7_t, time, k]=second_order([q0(7) q1(7)], qdi,[t(1) t(2)], step_time);
i=1;

q=q1;
Ti= directkinematic(robot.robot1, q);
    Pi = Ti(1:3,4)
    
    robot.robot1.equipment{1}.T0(1:3,4) = Pi;
    robot.robot1.equipment{1}.T0(2,4) = robot.robot1.equipment{1}.T0(2,4) + 0.095;
    robot.robot1.equipment{1}.T0(3,4) = robot.robot1.equipment{1}.T0(3,4) - 0.04
    
for n = time  
    qt = [q1_t(i) q2_t(i) q3_t(i) q4_t(i) q5_t(i) q6_t(i) q7_t(i)]; 
    i=i+1;
    qti = der2izq(qt);
    draw2robots(robot.robot1, qt, robot.robot2, qti);
end

robot.robot1.equipment{2}.graphical.has_graphics = 1;


t = 0;
q_traj = [q];
time = [t];
while t <= 2
    J = manipulator_jacobian(robot.robot1, q);
    
    Ti= directkinematic(robot.robot1, q);
    Pi = Ti(1:3,4);
    
    robot.robot1.equipment{1}.T0(1:3,4) = Pi;
    robot.robot1.equipment{1}.T0(2,4) = robot.robot1.equipment{1}.T0(2,4) + 0.095;
    robot.robot1.equipment{1}.T0(3,4) = robot.robot1.equipment{1}.T0(3,4) - 0.04;
   
    
    vx = 0.2*t^2 + -0.4*t + 23/60;
    vz = -19/20*t^2 + 33/20*t - 31/120;
    v = [vx 0 vz 0 0 0]';
    
    qd = compute_control_action_hierarchy(robot.robot1, v, q, '');
    
    qizq = der2izq(q);
    
    
    draw2robots(robot.robot1, q, robot.robot2, qizq)
    q = q + qd*step_time;
    
    t = t + step_time;
    q_traj = [q_traj q];
    time = [time t];
    
end

figure, xlabel('t (s)'), ylabel('q (rad)'), title('TRAYECTORIAS ARTICULARES'), hold on
plot(time, q_traj, 'r')




function error = error_position(Pf, Pi)
delta_P = Pf - Pi;
error = sqrt(delta_P(1)^2 + delta_P(2)^2 + delta_P(3)^2);

function error = error_orientation(Of, Oi)
delta_O = Of - Oi;
error = sqrt(delta_O(1)^2 + delta_O(2)^2 + delta_O(3)^2 + delta_O(4)^2);

function w = angular_w_between_quaternions(Q0, Q1, total_time)
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);
angle=2*atan2(len, Q(1));
%vector3 axis;
if len > 0
    axis=[Q(2) Q(3) Q(4)]./len;
else
    axis=[1 0 0];
end
w=axis*angle/total_time;

function [q_t, qd_t, qdd_t, time, k]=second_order(q, qd, t, delta_t)

k=inv([1 t(1) t(1)^2; 1 t(2) t(2)^2; 0 1 2*t(1)])*[q(1) q(2) qd(1)]';

time=t(1):delta_t:t(2);
q_t=k(1)+k(2)*time+k(3)*time.^2;
qd_t=k(2)*ones(1,length(time)) + 2*k(3)*time;
qdd_t=2*k(3)*ones(1,length(time));