%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This test compares the result of the inverse dynamic function using both:
%   a) the Newton-Euler algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all

q=[pi/4 pi/4]';
qd=[5 5]';
qdd = [6 6]';
g=[0 -9.81 0]';
%g=[0 0 0]';

fext = [10 41 1 1 1 1]';
%fext = [0 0 0 0 0 0]';

%Account for friction
robot.dynamics.friction = 1

figure, drawrobot3d(robot, q)
%robot=load_robot('example', '2dofplanar');
tau1 = inversedynamic(robot, q, qd, qdd, g, fext)

% Caution: in the next function g is a scalar value with sign in this function. 
% Let g=9.81 m/s^2 if gravity force acts on -Y0. Let g=-9.81 m/s^2 for the
% opposite direction.
tau2 = inversedynamics_2dofplanar(robot, q, qd, qdd, -sum(g), fext)

%compare results
error = tau1-tau2