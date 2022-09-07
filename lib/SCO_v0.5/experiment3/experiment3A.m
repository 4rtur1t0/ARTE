%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability index ALONG A LINE.
% Use stomp like to optimize along a surface/line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=experiment3A%(direction)
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
parameters.delta_mov=0.1;
%number of particles
%parameters.K = 10;
parameters.n_repeat = 3;
parameters.experiment_name = 'experiment3A.mat';
parameters.experiment_number = 3;

parameters.obstacles = [];

parameters.animate = 0;
close all
hfigures.hpaths = figure;
hfigures.hcosts = figure;
hfigures.hee = figure;
hfigures.htheta = figure;
hfigures.hbest_costs = figure;
hfigures.hdtheta = figure;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
create_experiment(parameters.experiment_number)
tic;
[Gout, random_manips] = initial_solutions_moore_penrose(robot, parameters.experiment_name);
elapsed = toc;
fprintf('TIC TOC: %g\n', elapsed/parameters.n_repeat);

figure, plot(random_manips')

 
