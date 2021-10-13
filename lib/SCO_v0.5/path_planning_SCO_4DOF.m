%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability index ALONG A LINE.
% Use stomp like to optimize along a surface/line
%
%
%  Please load 4 DOF ROBOT as
% 
%   robot = load_robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pk, final_manip]=path_planning_SCO_4DOF(K)
%function [pk, final_manip]=path_planning_SCO_4DOF%(K)%
%K = 5
close all;
global robot
global parameters
global hfigures

%STOMP PARAMETERS
%conversion from cost to Prob factor
parameters.lambda = .4;
parameters.lambda_manip = .4;
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
parameters.time_step=0.05;
%Error in XYZ to stop inverse kinematics
parameters.epsilonXYZ=0.001;
%Error in Quaternion to stop inverse kinematics.
parameters.epsilonQ=0.001;
parameters.stop_iterations=1000;

%number of waypoints
%parameters.N = 12;
parameters.delta_mov=0.1;

%number of particles
parameters.K = K;

parameters.obstacles = [];
parameters.trajectory = [];

parameters.animate = 0;
close all
hfigures.hpaths = figure;
hfigures.hcosts = figure;
hfigures.hee = figure;
hfigures.htheta = figure;
hfigures.hbest_costs = figure;
hfigures.hdtheta = figure;

% obstacles
% build obstacles based on a vector of advance and angle of advance
program_distances_angles_exp_II();

% build path
pathT = build_initial_path();

% LAUNCH SCO given the stored parameters
[pk, final_manip] = SCO_null_space(robot, pathT);



function di_phi = program_distances_angles_exp_II()
global parameters
% initial point for this experiment
pf = [-1.5, 0.5, 0]';
% this part programs the advance and angles which define obstacles
phi = pi/4;
di_phi = [phi, 1];
%number of interpolations between planes
n = 4;
for i=1:n
    phi = phi - 1/n*pi/4;
    di_phi = [di_phi; phi, parameters.delta_mov];    
end
di_phi = [di_phi; phi, 1];
for i=1:n
    phi = phi - 1/n*pi/4;
    di_phi = [di_phi; phi, parameters.delta_mov];    
end
di_phi = [di_phi; phi, 1];

for i=1:size(di_phi, 1)
    pf=dirobstacle(pf, di_phi(i, 2), di_phi(i, 1), i);
end

function di_phi = program_distances_angles_exp_III()
global parameters
% initial point for this experiment
pf = [-1.5, 0.5, 0]';
% this part programs the advance and angles which define obstacles
phi = pi/4;
di_phi = [phi, 1];
%number of interpolations between planes
n = 4;
for i=1:n
    phi = phi - 1/n*pi/4;
    di_phi = [di_phi; phi, parameters.delta_mov];    
end
di_phi = [di_phi; phi, 1];
for i=1:n
    phi = phi - 1/n*pi/4;
    di_phi = [di_phi; phi, parameters.delta_mov];    
end
di_phi = [di_phi; phi, 1];

for i=1:n
    phi = phi - 1/n*pi/4;
    di_phi = [di_phi; phi, parameters.delta_mov];    
end
di_phi = [di_phi; phi, 1];

for i=1:size(di_phi, 1)
    pf=dirobstacle(pf, di_phi(i, 2), di_phi(i, 1), i);
end

function pf = dirobstacle(p0, d, phi, k)
global parameters

u = [cos(phi), sin(phi), 0]';
%p0 = [x1 y1 0]';
pf = p0 + d*u;
p3 = pf + [0 0 1]';

% normal
n = compute_plane([p3 pf p0])
parameters.obstacles{k}.n = n;
parameters.obstacles{k}.p = p0;
% points defining surface. %define obstacles using point and normal
parameters.obstacles{k}.points = [p3 pf p0];
parameters.obstacles{k}.line = [p0 pf];
% define trajectory from plane
parameters.trajectory{k}.line = [p0 pf];
parameters.trajectory{k}.T0 = build_T_from_obstacle(parameters.obstacles{k}, p0);

% function obstacle1()
% global parameters
% 
% % LINE 1
% x1 = -1.5;
% y1 = .5; %m
% x2 = 0;
% y2 = 2; %m
% p0 = [x1 y1 0]';
% pf = [x2 y2 0]';
% p3 = [x2 y2 1]';
% 
% % normal
% n = compute_plane([p3 pf p0]);
% parameters.obstacles{1}.n = n;
% parameters.obstacles{1}.p = p0;
% % points defining surface. %define obstacles using point and normal
% parameters.obstacles{1}.points = [p3 pf p0];
% % define trajectory from plane
% parameters.trajectory{1}.line = [p0 pf];
% parameters.trajectory{1}.T0 = build_T_from_obstacle(parameters.obstacles{1}, p0);
% 
% 
% 
% function obstacle1_1()
% global parameters
% % LINE 1
% x1 = 0;
% y1 = 2; %m
% d = 0.1;
% p0 = [x1 y1 0]';
% pf = [x2 y2 0]';
% p3 = [x2 y2 1]';
% 
% % normal
% n = compute_plane([p3 pf p0]);
% parameters.obstacles{1}.n = n;
% parameters.obstacles{1}.p = p0;
% % points defining surface. %define obstacles using point and normal
% parameters.obstacles{1}.points = [p3 pf p0];
% % define trajectory from plane
% parameters.trajectory{1}.line = [p0 pf];
% parameters.trajectory{1}.T0 = build_T_from_obstacle(parameters.obstacles{1}, p0);
% 
% 
% 
% function obstacle2()
% global parameters
% %LINE 2
% x1 = 0;
% y1 = 2; %m
% x2 = 1;
% y2 = 2; %m
% p0 = [x1 y1 0]';
% pf = [x2 y2 0]';
% p3 = [x2 y2 1]';
% %normal
% n = compute_plane([p3 pf p0]);
% parameters.obstacles{2}.n = n;
% parameters.obstacles{2}.p = p0;
% parameters.obstacles{2}.points = [p3 pf p0];
% %define trajectory from plane
% parameters.trajectory{2}.line = [p0 pf];
% parameters.trajectory{2}.T0 = build_T_from_obstacle(parameters.obstacles{2}, p0);
% 
% function obstacle3()
% global parameters
% 
% %LINE 3
% x1 = 1;
% y1 = 2; %m
% x2 = 2;
% y2 = 0.5; %m
% p0 = [x1 y1 0]';
% pf = [x2 y2 0]';
% p3 = [x2 y2 1]';
% n = compute_plane([p3 pf p0]);
% parameters.obstacles{3}.n = n;
% parameters.obstacles{3}.p = p0;
% parameters.obstacles{3}.points = [p3 pf p0];
% %define trajectory from plane
% parameters.trajectory{3}.line = [p0 pf];
% parameters.trajectory{3}.T0 = build_T_from_obstacle(parameters.obstacles{3}, p0);

