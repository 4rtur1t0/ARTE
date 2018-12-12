%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute cost function to avoid obstacles.
% 
% Return:
% P: likelihood function
% q: the cost function it self
% p: the position of the end effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P] = cost_function_distance_7dof(theta)
global robot parameters
lambda_obstacle = parameters.lambda_obstacles;
%T = directkinematic(robot, theta(:));
%given q1 is known, compute first DH transformation
A01 = dh(robot, theta(:), 1);
A12 = dh(robot, theta(:), 2);
A23 = dh(robot, theta(:), 3);
A34 = dh(robot, theta(:), 4);
A45 = dh(robot, theta(:), 5);
A56 = dh(robot, theta(:), 6);
%A67 = dh(robot, theta(:), 7);

T2 = A01*A12;
T3 = T2*A23;
T4 = T3*A34;
T5 = T4*A45;
T6 = T5*A56;
%T7 = T5*A67;

%the position of systems 1, 2, and 3
p00 = [0 0 0]';
p01 = A01(1:3, 4);
p02 = T2(1:3, 4);
p03 = T3(1:3, 4);
p04 = T4(1:3, 4);
p05 = T5(1:3, 4);
p06 = T6(1:3, 4);
%p04 = T4(1:3, 4);

%find the min distance of each point to all the obstacles
d1 = min_distance_to_obstacles(p01);
d2 = min_distance_to_obstacles(p02);
d3 = min_distance_to_obstacles(p03);
%this is just a check--> d4 must be zero always
d4 = min_distance_to_obstacles(p04);
d5 = min_distance_to_obstacles(p05);
d6 = min_distance_to_obstacles(p06);

vector_points = [p00 p01 p02 p03 p04 p05 p06];

%compute self distances between the points
d_auto_collisions = self_distances(vector_points);

P1 = distance_weight(d1);
P2 = distance_weight(d2);
P3 = distance_weight(d3);
P4 = distance_weight(d4);
P5 = distance_weight(d5);
P6 = distance_weight(d6);

%self collisions weight
Pauto = distance_weight(d_auto_collisions);

%return likelihood
P = P1*P2*P3*P4*P5*P6*Pauto;



%
% Computes the min distance of the point to all the 
% objects in the scene.
%
function dist = min_distance_to_obstacles(point)
global parameters
%pos obstacle
obstacles = parameters.obstacles;
dists = [];
%the min of min distances
for o=1:size(obstacles,2)
   obstacle = obstacles{o};
   %obtain plane
   distL = shortest_distance_plane_constrained(obstacle, point);
   dists = [dists distL];
end
dist = min(dists);

function w=distance_weight(dist)
global parameters
%epsilon = parameters.epsilon;
x = max(parameters.epsilon_distance-dist,0);
w=exp(-x/parameters.lambda_obstacles);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Return signed distance to plane!
% The plane is defined by the normal n and a point p belonging
% to the plane. Compute the distance to a point point
% if the point does not lie within the boundaries of the plane defined by
% three points, then, return the closest distance to the three lines that
% define the plane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = shortest_distance_plane_constrained(obstacle, point)
n = obstacle.n;
a = n(1);
b = n(2);
c = n(3);
p = obstacle.p;
x0 = p(1);
y0 = p(2);
z0 = p(3);
%compute free term d
d = -(a*x0 + b*y0 + c*z0);
%the point to test
%compute distance of plane to this point.
x = point(1);
y = point(2);
z = point(3);
%project p onto the plane
p = project_to_plane(obstacle, point);

%is p within ostacle boundaries
%this means that the projection of the point to the plane
%is within the three points defined by the plane
in_bounds=is_in_boundary(obstacle, p);

if in_bounds
    %compute min distance to plane
    D=(a*x + b*y + c*z + d)/(sqrt(a^2 + b^2 + c^2));
else
    p1 = obstacle.points(:,1);
    p2 = obstacle.points(:,2); 
    p3 = obstacle.points(:,3); 
    %return min distance to any of the lines of the obstacle
    d1 = min_distance_to_line(p1, p2, point);
    d2 = min_distance_to_line(p2, p3, point);
    d3 = min_distance_to_line(p1, p3, point);
    D = min([d1 d2 d3]);
end



%Returns the min distance of the distance to all self_collisions
function d_auto_collisions = self_distances(robot_points)
%compute all possible combinations
dists =[];
for i=1:size(robot_points,2)
    d=[];
    for j=1:size(robot_points,2)
        if j==i
            continue
        end
        d = [d dist(robot_points(:,i),robot_points(:,j))];
    end
    dists = [dists min(d)];
end
d_auto_collisions = min(dists);

function d=dist(pa, pb)
d=sqrt((pa-pb)'*(pa-pb));


%returns the point
function p_proj = project_to_plane(obstacle, point)
n = obstacle.n;
a = n(1);
b = n(2);
c = n(3);
p = obstacle.p;
x0 = p(1);
y0 = p(2);
z0 = p(3);
%compute free term d
d = -(a*x0 + b*y0 + c*z0);

%the point to project
px = point(1);
py = point(2);
pz = point(3);

t=(-d-a*px-b*py-c*pz)/(a^2+b^2+c^2);

p_proj(1) = px + a*t;
p_proj(2) = py + b*t;
p_proj(3) = pz + c*t;
p_proj = p_proj(:);

%is p within ostacle boundaries
%tells whether the point p that belongs to the infinite plane is 
%within the bounds of the 3 points that define the obstacle.
function in_bounds=is_in_boundary(obstacle, point)
%yes! both test_a and test_b
%point = [-0.2 0.3 0]'; %PASSED
%no! both test_a ok but test_b no 
%point = 3*[-0.2 0.3 0]'; % PASSED
%no! test_b, ok but test_a no
%point = [-0.4 0 0]';

n = obstacle.n;

p1 = obstacle.points(:,1);
p2 = obstacle.points(:,2); 
p3 = obstacle.points(:,3); 
%n = cross(p2-p1, p3-p1);
%within
va = p2 - p1;
vb = p3 - p1;
vp = point - p1;
test_a = within_two(n, va, vb, vp);

va = p1 - p2;
vb = p3 - p2;
vp = point - p2;
test_b = within_two(n, va, vb, vp);
if test_a==1 && test_b == 1 
    in_bounds = 1;
else
    in_bounds = 0;
end

function in_bounds = within_two(n, va, vb, vp)
ca = cross(va, vp)'*n;
cb = cross(vp, vb)'*n;
if ca*cb > 0
    in_bounds = 1;
else
    in_bounds = 0;
end


%
%line defined by two points x1 and x2
%compute shortest distance to point
function dist = min_distance_to_line(x1, x2, point)
dist = norm(cross((point-x1), (point-x2)))/norm(x2-x1);
    
    
%     % line defined by p1 and p2
% % P1=(x1,y1) and P2=(x2,y2) then the distance of (x0,y0) from the line is:
% function dist = shortest_distance_line(p1, p2, pcon)
% x1 = p1(1);
% y1 = p1(2);
% x2 = p2(1);
% y2 = p2(2);
% 
% x0 = pcon(1);
% y0 = pcon(2);
% dist=abs((y2-y1)*x0-(x2-x1)*y0 + x2*y1-y2*x1)/sqrt((y2-y1)^2+(x2-x1)^2);
% 
% %now compute the sign
% m = (y2-y1)/(x2-x1);
% y = m*(x0-x1)+y1;
% 
% %if y0 is below, then outside obstacle
% if y >= y0
%     signo = 1;
% else %inside obstacle dist is negative
%     signo = -1;
% end
% dist = dist*signo;
