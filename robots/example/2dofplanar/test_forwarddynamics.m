%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This test compares the result of the inverse dynamic function using both:
%   a) the Newton-Euler algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test_forwarddynamics()
close all
global robot g q0 qd0 tau total_simulation_time

g=[0  -9.81 0]'; %y0 axis
%initial position and joint speed
q0 = [0 0]';
qd0 = [0 0]';
tau = [0 0]';%no torques applied
total_simulation_time = 5;
%select friction or not
robot.dynamics.friction = 1;

fprintf('\nTHE SIMULATION PRESENTS THE ROBOT AT AN INITIAL POSITION WHEN NO TORQUES ARE APPLIED\n')

%load robot parameters
robot=load_robot('example', '2dofplanar');

% drawrobot3d(robot, q0);
% adjust_view(robot);

[t q qd] = forward_test_A(); 

figure, plot(t, q, 'r'), hold

[t q qd] = forward_test_B(); 

plot(t, q, 'b')

speed = 10
animate(robot,[q(1,1:speed:length(q)); q(2,1:speed:length(q))])



function [t q qd] = forward_test_A()
global robot g q0 qd0 tau total_simulation_time

fprintf('\nCOMPUTING FORWARD DYNAMICS (this may take a while)')

%this may take a while, since it requires integration
%of the acceleration at each time step
[t q qd] = forwarddynamic(robot, total_simulation_time, q0, qd0, tau, g, []);


function [t q qd] = forward_test_B()
global total_simulation_time

[t, y] = runge_kutta(@forward_dynamic_robot2, [0 0 0 0]', [0 total_simulation_time], 0.01);

%Assign joint variables and joint speeds.
q = y(1:2,:);
qd = y(3:4,:);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to simulate the movement of a 2 DOF robot
% Called from function exerciseD()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = forward_dynamic_robot2(t, y)
global tau g robot
%we must return the solution of
% [dx1/dt; dx2/dt], in this case [qd; qdd]
t
qdd=forwarddynamics_2dofplanar(robot, y(1:2,1), y(3:4,1), tau, -sum(g), [0 0 0 0 0 0]);
xd = [y(3:4,1); qdd];


