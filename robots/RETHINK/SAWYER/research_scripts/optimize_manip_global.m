
%   MAXIMIZE manipulability along the null space
% use a simple delta manip to stop.
% whenever the increase in manipulability is below a threshold, stop
%
%  USES A GLOBAL search technique
function [optimq, optimw] = optimize_manip_global%(robot, q0, objective)
q0=[0.1 0.4 0.8 1.6 0.3 0.5 0.6]'
global robot
q = q0;
T = directkinematic(robot, q);
drawrobot3d(robot, q)
%move along q to increase manipulability at a global max
alpha = 0.1;
ws = [];
qq = [q];
suma=0;
i=0;
%qda = [1 1 1 1 1 1 1]';
qda = [0 0 1 0 0 0 0]';
qqd = [];
ddet = [];
hqd = figure;
hdet = figure;
dd=0;
while i < 1000
    %qda = [rand rand rand rand rand rand rand]';
    J = manipulator_jacobian(robot, q);
    ws = compute_manip(robot, q);
    fprintf('current manip is %f:\n', ws)
    I = eye(7);
    Jp = pinv(J);
    qd_null = (I-Jp*J)*qda;
    qd_null = qd_null;%/norm(qd_null);
    if norm(qd_null)<0.01 && dd==0
        qda=-qda;
        dd=1;
    end
    %si nos movemos sobre el null space no debemos producir movimiento
    %sobre el end effector
    q = q + 0.1*qd_null;
    %Ti = directkinematic(robot, q);
    qq = [qq q];
    qqd =[qqd qd_null];
    ddet = [ddet det(J*J')];
    i=i+1;
    drawrobot3d(robot, q)
    
    figure(hqd)
    plot(qqd')
    figure(hdet)
    plot(ddet)
end
ws = compute_manip(robot, qq);
figure, plot(ws)

figure, plot(qq')

for i=1:10:size(qq,2)
      drawrobot3d(robot, qq(:,i))
      draw_axes(T, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);     
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