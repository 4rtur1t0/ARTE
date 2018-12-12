%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function experiment2A
close all;
global robot
global parameters
global hfigures

%STOMP PARAMETERS
%conversion from cost to Prob factor
parameters.lambda = .4;
parameters.lambda_obstacles = .2;
%height of the obstacle
parameters.yo = 2.5;
%cost function starts at this distance
%must be below 0.3 for the 4 DOF robot
parameters.epsilon = 0.2;
%multiply noise by this facto
%parameters.noise_k = 5;
%parameters.noise_sigma_null_space = 0.01;
parameters.alpha=0.02;
parameters.time_step=0.01;

%number of waypoints
parameters.N = 12;
%number of particles
parameters.n_repeat = 2000;
parameters.experiment_name = 'experiment2A_m.mat';

parameters.animate = 0;
close all
hfigures.hpaths = figure;
hfigures.hcosts = figure;
hfigures.hee = figure;
hfigures.htheta = figure;
hfigures.hbest_costs = figure;
hfigures.hdtheta = figure;

parameters.obstacles = [];
%LINE 1
x1 = -1.5;
y1 = .5; %m
x2 = 0;
y2 = 1.5; %m
phi = 3*pi/4; 
p0 = [x1 y1 0]';
pf = [x2 y2 0]';
T0 = build_T_4dof(p0, phi);
parameters.obstacles{1}.line = [p0 pf];
parameters.obstacles{1}.T0 = T0;

%LINE 2
x1 = 0;
y1 = 1.5; %m
x2 = 1;
y2 = 1.5; %m
phi = pi/2; 
p0 = [x1 y1 0]';
pf = [x2 y2 0]';
T0 = build_T_4dof(p0, phi);
parameters.obstacles{2}.line = [p0 pf];
parameters.obstacles{2}.T0 = T0;

%LINE 3
x1 = 1;
y1 = 1.5; %m
x2 = 2;
y2 = 0.5; %m
phi = pi/4; 
p0 = [x1 y1 0]';
pf = [x2 y2 0]';
T0 = build_T_4dof(p0, phi);
parameters.obstacles{3}.line = [p0 pf];
parameters.obstacles{3}.T0 = T0;
%(robot, T0, p0, pf)
[Gout, random_manips] = initial_solutions_moore_penrose(robot,parameters.experiment_name);


function T = build_T_4dof(p, phi)
T = [cos(phi) -sin(phi) 0 p(1);
     sin(phi) cos(phi) 0 p(2);
     0            0     1  p(3);
     0             0    0   1];
 
 function T = build_T_sawyer(p, phi)
T = [1  -sin(phi) 0 p(1);
     0  cos(phi) 0 p(2);
     0            0     1  p(3);
     0             0    0   1];
 



 
