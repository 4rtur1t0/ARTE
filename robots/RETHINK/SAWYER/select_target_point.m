% Simple target point selection for simulation
function [Q, P] = select_target_point(robot, qf)
close all;

%fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds and viceversa')
%robot=load_robot('RETHINK','SAWYER');
%qf = [0.0 -0.6 0.0 1.2 0.0 1.0 0.0]';
%That is a quaternion to T transformation which is trivial
%quaternion2T(Q,P) --> cuaternion plus Px Py Pz
Tf=directkinematic(robot, qf);
drawrobot3d(robot, qf)

%and now transform by the Trel (S7-end tool)


%this is the final transformation of the piece ref system
Tf=Tf*robot.Tcoupling

draw_axes(Tf, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);

%Now represent as Quaternion/P
Q = T2quaternion(Tf)
P = Tf(1:3,4)




