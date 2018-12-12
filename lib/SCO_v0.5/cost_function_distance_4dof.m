%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute cost function to avoid obstacles.
% 
% Return:
% P: likelihood function
% q: the cost function it self
% p: the position of the end effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P] = cost_function_distance_4dof(theta)
global robot parameters
lambda_obstacle = parameters.lambda_obstacles;
T = directkinematic(robot, theta(:));
%given q1 is known, compute first DH transformation
A01 = dh(robot, theta(:), 1);
A12 = dh(robot, theta(:), 2);
A23 = dh(robot, theta(:), 3);
T2 = A01*A12;
T3 = A01*A12*A23;
%the position of systems 1, 2, and 3
p00 = [0 0 0]';
p01 = A01(1:3, 4);
p02 = T2(1:3, 4);
p03 = T3(1:3, 4);
p04 = T(1:3, 4);

%find the min distance of each point to all the obstacles
d1 = min_distance_to_obstacles(p01);
d2 = min_distance_to_obstacles(p02);
d3 = min_distance_to_obstacles(p03);
%this is just a check--> d4 must be zero always
d4 = min_distance_to_obstacles(p04);

%compute self distances between the points
d_auto_collisions = self_distances(p00, p01, p02, p03);

% drawrobot3d(robot, theta)
% line_work = parameters.obstacles{1}.line;
% plot(line_work(1,:),line_work(2,:))

P1 = distance_weight(d1);
P2 = distance_weight(d2);
P3 = distance_weight(d3);
%self collisions weight
P4 = distance_weight(d_auto_collisions);

%return likelihood
P = P1*P2*P3*P4;



%
% Computes the min distance of the point to all the 
% objects in the scene.
%
function dist = min_distance_to_obstacles(point)
global parameters
%pos obstacle
obstacles = parameters.obstacles;
%x = p(1);
%y = p(2);
dists = [];
%the min of min distances
for o=1:size(obstacles,2)
   obstacle = obstacles{o};
   p1 = obstacle.line(:,1);
   p2 = obstacle.line(:,2);
   distL = shortest_distance(p1, p2, point);
   dists = [dists distL];
end
dist = min(dists);

function w=distance_weight(dist)
global parameters
%epsilon = parameters.epsilon;
x = max(parameters.epsilon-dist,0);
w=exp(-x/parameters.lambda_obstacles);

% function r = uniform(a, b, rows, columns)
% r =  a + (b-a).*rand(rows,columns);

% line defined by p1 and p2
% P1=(x1,y1) and P2=(x2,y2) then the distance of (x0,y0) from the line is:
% 
% figure, 
% plot(parameters.obstacles{1}.line(1,:),parameters.obstacles{1}.line(2,:))
function dist = shortest_distance(p1, p2, pcon)
x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);

x0 = pcon(1);
y0 = pcon(2);
dist=abs((y2-y1)*x0-(x2-x1)*y0 + x2*y1-y2*x1)/sqrt((y2-y1)^2+(x2-x1)^2);

%now compute the sign
m = (y2-y1)/(x2-x1);
y = m*(x0-x1)+y1;

%if y0 is below, then outside obstacle
if y >= y0
    signo = 1;
else %inside obstacle dist is negative
    signo = -1;
end
dist = dist*signo;


function d_auto_collisions = self_distances(p00, p01, p02, p03)
%just the combinations
%squared distance
%feasible combinations
d0 = dist(p00,p02);
d1 = dist(p00,p03);
d2 = dist(p01,p03);

d_auto_collisions = min([d0 d1 d2]);
d_auto_collisions = sqrt(d_auto_collisions);

function d=dist(pa, pb)
d=(pa-pb)'*(pa-pb);

