

function compute_workspace
global robot
close all
M = 10000;
work = [];
for i=1:M,
    i
    q = randInRange();
    T = directkinematic(robot, q);
    p = T(1:3,4);
    work = [work; p'];
end
figure,
plot(work(:,1), work(:,2), 'k.')


function q = randInRange()
global robot
q=[];
for i=1:robot.DOF,
    maxrange = robot.maxangle(i,2);
    minrange = robot.maxangle(i,1);
    wide = maxrange-minrange;
    qi = minrange + wide*rand();
    q = [q qi];
end
q=q(:);