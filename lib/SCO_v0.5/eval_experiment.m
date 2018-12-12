%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [correct_manips]=eval_experiment(G, experiment_manips)
%load('/Users/arturogilaparicio/Desktop/arte/robots/RETHINK/sgo_v0.5/experiment1/experiment1B/experiment1B_K_1_N_50.mat')
global robot
global parameters
parameters.check_collision_weight=0.85;

%data in experiment1B
%Eval success rate
success = 0;
total_tries = 0;
s_manip = [];
correct_manips = [];
for i=1:size(G,2)
    pathq = G{i}.pathq;
    coll = check_collisions(pathq);
    
    if coll==0
        success = success + 1;
        s_manip = [s_manip mean(experiment_manips(i,:))];
        correct_manips = [correct_manips; experiment_manips(i,:)];
    else
        %animate_local(robot, pathq)
    end
    total_tries = total_tries + 1;
end
success_rate = success/total_tries;
mean_manip_integral = mean(s_manip);
cov_manip_integral= cov(s_manip);
max_manip_integral = max(s_manip);
fprintf('Total tries: %d\n', total_tries)
fprintf('Total successful trajectories: %d\n', success)
fprintf('Success rate: %f\n', success_rate)
fprintf('Mean manip integral: %f +- %f\n', mean_manip_integral, 2*sqrt(cov_manip_integral))
fprintf('Max manip integral: %f\n', max_manip_integral)




 



 
