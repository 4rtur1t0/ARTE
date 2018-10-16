
%   JUST USE A GLOBAL SEARCH AROUND Q0 TO STABLISH
%   THE BEST MANIPULABILITY FOR THE ARM
%
%  USES A GLOBAL search technique
function [optimq, optimw] = optimize_manip_global_mcl(robot, q0, Ti, objective)
mcl_qq = [];
qq = [];
tt=[];
%generate 1000 random poses around q0
M = 1000;
%generating with standard deviation, and getting at +-2*sigma --> +-pi for
%each joint
%Max global manipulability measure
sigma=pi/2;

q0=[0 0 0 0 0 0 0];
for i=1:M
    %q = q0 + [normrnd(0, sigma, 1, 7)]';
    q = [normrnd(0, sigma, 1, 7)]';
    %use each of them as seed for inverse kinematics
    mcl_qq = [mcl_qq q];
end

ws = compute_manip(robot, mcl_qq);
%just retrieve
figure, plot(ws)
fprintf('max manip absolute for the robot is')
[y, index]=max(ws);
max_manip = y
fprintf('at pose') 
q=mcl_qq(:, index)

q = optimize_manip_local(robot, q, 'max_manip')
fprintf('pose optimized to')
q
drawrobot3d(robot, q)

%ends with 1000 solutions of the inverse kinematics
%use each as a start point for manipulability maximization
% for all particles
for i=1:M %i < 1000
    q = mcl_qq(:,i);
    %for each robot pose
    %trigger the inverse kinematics to Ti
    %this finds the closest inverse kinematics solution to the previous q
    q = inverse_kinematics_sawyer(robot, Ti, q, 'moore_penrose');
    %move along the 
    q = optimize_manip_local(robot, q, 'max_manip');
    %truncate to -pi, pi
    q=atan2(sin(q), cos(q));
    qq = [qq q];
end
ws = compute_manip(robot, qq);
%just retrieve
figure, plot(ws)

for i=1:1:size(qq,2)
      drawrobot3d(robot, qq(:,i))
      draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);     
end

if strcmp(objective, 'max_manip')
    [y,j] = max(ws);
else
    [y,j] = min(ws);
end

optimq = qq(:,j);
optimw = y;

%truncate to -pi, pi
optimq=atan2(sin(optimq), cos(optimq));
%optimw = compute_manip(robot, optimq);




% for i=1:size(qq,2)
%      drawrobot3d(robot, qq(:,i))
%      draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);     
% end