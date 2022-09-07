%
% FIND K INITIAL SOLUTIONS ALONG A PATH USING Moore-penrose
% Selecting K as brute Force.
% The initial q are selected randomly. A solution is found on the path
% using pseudo-inverse.
%
% - EXPERIMENT COMPUTES A VANILLA SOLUTION PATH USING A BRUTE FORCE SOLUTION
% - THOSE PATHS IN COLLISION ARE DISCARDED.
% 
% RESULT: THE result may show
function [Gout, random_manips] = initial_solutions_moore_penrose(robot, experiment_name)
global parameters

random_manips = [];
Gout={};
for k=1:parameters.n_repeat
  q0 = uniform(-pi, pi, 1, robot.DOF)';
  [pathq, pathT] = plan_path_moore_penrose(robot, q0);
  %generate particle
  Gout{k}.pathq = pathq;
  Gout{k}.pathT = pathT;

  %manips = compute_manip_4dof(robot, pathq);
  manips = compute_manip(robot, pathq);
  random_manips = [random_manips; manips];
  save(experiment_name)
end


%
% Uniform value betweenn min_val and max_val
%
function delta = uniform(min_val, max_val, n, m)
delta = (max_val-min_val)*rand(n, m) + min_val;



