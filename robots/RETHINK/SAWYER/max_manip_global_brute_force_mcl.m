% MAX MANIP AT SAWYER
% M = 15000
% pi/2--> 0.2333
% M = 15000
% sigma=pi -- 0.2441
%0.2373
%   Use MCL to obtain the maximum value for manipulability in the robot
function [optimq, optimw] = max_manip_global_brute_force_mcl%(robot, M)
global robot
%generate 1000 random poses around q0
%M = 1;
M = 20000;
%generating with standard deviation, and getting at +-2*sigma --> +-pi for
%each joint
%Max global manipulability measure
sigma=pi;
mcl_qq = [];

q0=[0 0 0 0 0 0 0]';
%q0 = [-1.3 -0.7 0.0 1.8 0.3 1.0 0.3]';
for i=1:M
    i
    %q = q0 + [normrnd(0, sigma, 1, 7)]';
    %q = q0 + [normrnd(0, sigma, 1, 7)]';
    q = q0 + [uniform(-pi, pi, 1, 7)]';
    %w1=compute_manip(robot, q);
    %q = optimize_manip_local(robot, q, 'max_manip', 1e-5);
    mcl_qq = [mcl_qq q];
    %w2=compute_manip(robot, q);
    %fprintf('Total manip is: %f, Increased manip is: %f', w2, w2-w1)
end

figure, plot(mcl_qq(1,:))
ws = compute_manip(robot, mcl_qq);
%just retrieve
figure, plot(ws)
fprintf('max manip absolute for the robot is')
[y, index]=max(ws);
max_manip = y
fprintf('at pose') 
q=mcl_qq(:, index)

q = optimize_manip_local(robot, q, 'max_manip', 1e-5)
fprintf('pose optimized to')
q
max_manip = compute_manip(robot, q)
drawrobot3d(robot, q)


function r = uniform(a, b, rows, columns)
r =  a + (b-a).*rand(rows,columns);
