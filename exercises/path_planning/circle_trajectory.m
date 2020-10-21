
function circle_trajectory
close all 
figure
% points belonging to trajectory p1-->p2-->p3     
[p1, p2, p3] = rand3points();
[center,radius,v1,v2] = circlefit3d(p1',p2',p3');

plot3(p1(1),p1(2),p1(3),'ro');
hold on;
plot3(p2(1),p2(2),p2(3),'go');
plot3(p3(1),p3(2),p3(3),'bo');

%build up unit vectors
vp1 = p1(:)-center(:);
vp1 = vp1/norm(vp1);
vp2 = p2(:)-center(:);
vp2 = vp2/norm(vp2);
vp3 = p3(:)-center(:);
vp3 = vp3/norm(vp3);
% draw vectors vp1, vp2 and vp3
draw_my_vector(radius*vp1', center', 'vp1')
draw_my_vector(radius*vp2', center', 'vp2')
draw_my_vector(radius*vp3', center', 'vp3')

% draw ortonormal vectors v1 and v2 on the trajectory plane      
draw_my_vector(radius*v1', center', 'v1')
draw_my_vector(radius*v2', center', 'v2')

% find angles theta with respect to v1
th1 = find_theta(v1, v2, vp1);
th2 = find_theta(v1, v2, vp2);
th3 = find_theta(v1, v2, vp3);

% go from th1 to th2 and then to th3
a1 = th1:0.01:th2;
a2 = th2:0.01:th3;
b = [a1 a2];
      
for i=1:length(b)
    a = b(i);
    p = center(:) + radius*v1(:)*cos(a) + radius*v2(:)*sin(a);
    plot3(p(1),p(2), p(3),'r.');
end
axis equal;grid on;rotate3d on;

function theta = find_theta(v1, v2, vp)
cth = dot(v1, vp);
sth = dot(v2, vp);
theta = atan2(sth, cth);

function draw_my_vector(V, p0, text_label)
p1 = p0(:) + V(:);
vect_arrow(p0, p1, 'k', 2) 
text(p1(1)+0.0005, p1(2)+0.0005, p1(3)+0.0005, text_label, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', 15); 


function [p1, p2, p3]=rand3points()
    
p1=rand(1,3);
center=rand(1,3);
vp1 = p1-center;
radius = norm(vp1);
vp1 = vp1/radius;
vp1 = vp1(:);
theta1 = pi/4*rand;
theta2 = pi/4*rand;
R1 = Rot(theta1, 'Z');
R2 = Rot(theta2, 'X');
p1 = p1(:);
p2=center(:)+radius*R1*vp1(:);
p3=center(:)+radius*R1*R2*vp1(:);
       
