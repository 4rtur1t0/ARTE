%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability index ALONG A LINE.
% Use stomp like to optimize along a surface/line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function experiment2B_K5_N30
close all;
global parameters

%STOMP PARAMETERS
%number of particles
K = 5;

%repeat experiment number of times
parameters.n_repeat = 30;
parameters.experiment_name = 'experiment2B_K5_N30.mat';
parameters.animate = 0;

%repeat the experiment E times
random_manips=[];
Gout = [];
for i=1:parameters.n_repeat
    close all
    [pk, final_manip] = path_planning_SCO_4DOF(K);
    Gout{i}=pk;
    random_manips = [random_manips; final_manip];
    save(parameters.experiment_name)
end


'ended'
parameters.experiment_name
 
