
function simple_control_jacobian_sawyer%(robot)
% SIMPLE CONTROL FOR SAWYER BASED ON JACOBIAN 
close all;
global robot
global control_params
%used to alig polyshing tool
control_params.kx = 0.1;
control_params.ky = 0.1;
%use for more/less pressure
control_params.kz = 0.1;
%sample time
control_params.T = 100; %Hz

%in the case of a contact, 
%reference forces and moments
fref = [0 0 15];
mref = [0 0 0];

%measured forces and moments
%ft is positive--> tries to go down to get contact
ft = [0 0 15];
mt = [0 3 0];
%initial pose
q = [0.0 -0.6 0.0 1.2 0.0 0.7 0.0]'; 
%qd_1 = [0 1 0 0 0 0 0 ]';

%fprintf('\nThe demo shows how to compute the end effectors speed as a function of the joint speeds and viceversa')
%robot=load_robot('RETHINK','SAWYER');

drawrobot3d(robot, q)
%view qd apply in a loop. Assuming that the forces do not change
%just to try what the end effector does
qq=[];
pp=[];
for i=1:160,
    %based on ft and mt and the references, try to compute a basic control
    %action (up level)
    %v, w are computed in the 7th system reference system
    [v, w] = compute_basic_control_action(ft, mt, fref, mref);
    T = directkinematic(robot, q);
    %change from local to global coordinates
    %rotate to the global frame mate!
    v0 = T(1:3,1:3)*v;
    w0 = T(1:3,1:3)*w;
    %this is the control law in terms of global or high level!
    Vref = [v0' w0']';
    %the low level control action is:
    %should be interesting to compute with an optimization scheme
    %qd = compute_joint_velocity(robot, q, [v0 w0]');
    %3 is the redundant joint
    qd = compute_joint_velocity_redundant(robot, q, Vref, 3);
    %informative
    V = compute_end_velocity(robot, q, qd);
    V-Vref

    q=q+qd/control_params.T;

    qq = [qq q];
    pp = [pp T(1:3,4)];
end

%animate_local(robot, qq)

drawrobot3d(robot, qq(:,1))
pause(0.1);   
drawrobot3d(robot, qq(:,end))

figure, hold
plot(pp(1, :), 'r')
plot(pp(2, :), 'g')
plot(pp(3, :), 'b')
legend('p_x', 'p_y', 'p_z')
title('End effector position (m)')
xlabel('time (s)')




%Ojo, ft medido en el sistema de coordenadas del extremo.
%comprobar que si frefz>0, 
%v, w are computed coordinates relative to the end tool reference frame
% S7
function [v, w] = compute_basic_control_action(ft, mt, fref, mref)
global control_params
%calculamos v y w en coordenadas relativas de la herramienta
vz = -control_params.kz*(ft(3)-fref(3));
v = [0 0 vz];
wx=-control_params.kx*(mt(1)-mref(1));
wy=-control_params.ky*(mt(2)-mref(2));
w =[wx wy 0];
v = v(:);
w = w(:);

function vq = compute_joint_velocity_redundant(robot, q, V, joint_0)

J = manipulator_jacobian(robot, q);  

%Get J squared, remove redundant column
J(:,joint_0)=[];

%assure V is a column vector
V = V(:);
%Solve inverse kinematic problem with the inversion of J
vq = J\V;%inv(J)*V';
%return the whole solution
vq = [vq(1:joint_0-1); 0; vq(joint_0:end)];
%vq2 = inv(J)*V;
%vq3 = inv(J'*J)*J'*V


function animate_local(robot, q)
global configuration 

h=figure(configuration.figure.robot);, hold on,

%get adjusted view
[az,el] = view;
for j=1:size(q, 2),
    clf(h);
    qj=q(:,j);  
    view(az,el);
   
    drawrobot3d(robot, qj);  
    
    %pause to get a nice view
    pause(0.1);   
end
