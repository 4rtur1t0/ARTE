
function create_experiment(experiment_number)
global parameters

if experiment_number == 1
    program_distances_angles_exp_I()
elseif experiment_number == 2
        program_distances_angles_exp_II()
else
        program_distances_angles_exp_III()
end

 


function di_phi = program_distances_angles_exp_I()
global parameters
% initial point for this experiment
pf = [-1.5, 0.5, 0]';
% this part programs the advance and angles which define obstacles
phi = pi/4;
di_phi = [phi, 3];

for i=1:size(di_phi, 1)
    pf=dirobstacle(pf, di_phi(i, 2), di_phi(i, 1), i);
end


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