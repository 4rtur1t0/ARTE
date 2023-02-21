function circle_trajectory %(p1, p2, p3)


close all

p1=rand(1,3);p2=rand(1,3);p3=rand(1,3);
[pc,r,v1,v2] = circlefit3d(p1,p2,p3);
plot3(p1(:,1),p1(:,2),p1(:,3),'ro');hold on;plot3(p2(:,1),p2(:,2),p2(:,3),'go');plot3(p3(:,1),p3(:,2),p3(:,3),'bo');

x1 = (p1-pc);
x1 = x1/norm(x1);
phi1 = dot(v1,x1);
x2 = (p2-pc);
x2 = x2/norm(x2);
phi2 = dot(v1,x2);

angle = [phi1 phi2];
%angle = 0:0.1:2*pi;
%angle = pi/4:0.1:pi/2;
for i=1:length(angle)
        a = angle(i);
        x = pc(:,1)+cos(a)*r.*v1(:,1)+sin(a)*r.*v2(:,1);
        y = pc(:,2)+cos(a)*r.*v1(:,2)+sin(a)*r.*v2(:,2);
        z = pc(:,3)+cos(a)*r.*v1(:,3)+sin(a)*r.*v2(:,3);
        plot3(x,y,z,'r+');
end
axis equal;grid on;rotate3d on;

% fit a circle with three points

%[pm, r, v1, v2] = circlefit3d(p1', p2', p3');
