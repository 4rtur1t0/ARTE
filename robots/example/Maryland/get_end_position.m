%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position of the end effector of a Delta 3DOF robot.
% 
% [X,Y,Z]=get_end_position(P1,P2,P3,L2)
% P1, P2 and P3 are three points in the space.
% L2 is the radius of the sphere, which correspond to the length of the 
% lower arms in the Delta 3DOF robot.
%
% The function computes the intersection of three spheres of radius L2 that
% are centred at points P1, P2 and P3 in space.
% 
% Returns: the two posibles solutions for the position effector.
% 
% 
%   Author: Ángel Rodríguez
%   Date: 18/12/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,Y,Z]=get_end_position(P1, P2, P3, L2)

x1=P1(1);
y1=P1(2);
z1=P1(3);
x2=P2(1);
y2=P2(2);
z2=P2(3);
x3=P3(1);
y3=P3(2);
z3=P3(3);

%Constants needed in the system solve

c1=(x1^2 + y1^2 + z1^2);
c2=(x2^2 + y2^2 + z2^2);
c3=(x3^2 + y3^2 + z3^2);

d1=((z3-z2)*(y2-y1)-(z2-z1)*(y3-y2))/((y3-y2)*(x2-x1)-(x3-x2)*(y2-y1));
d2=((c2-c1)*(y3-y2)-(c3-c2)*(y2-y1))/(2*((y3-y2)*(x2-x1)-(x3-x2)*(y2-y1)));
d3=((z2-z1)*(x3-x2)-(z3-z2)*(x2-x1))/((y3-y2)*(x2-x1)-(x3-x2)*(y2-y1));
d4=((c3-c2)*(x2-x1)-(c2-c1)*(x3-x2))/(2*((y3-y2)*(x2-x1)-(x3-x2)*(y2-y1)));

%Parameters os the second degree equation to obtain Z

a=(d2-x1)^2 + (d4-y1)^2 + z1^2 - L2^2;
b=2*(d1*(d2-x1) + d3*(d4-y1) - z1);
c=d1^2+d3^2 +1;

root=sqrt(b^2 - 4*a*c);

%Solution

Z(1)=(-b+root)/(2*c);
Z(2)=(-b-root)/(2*c);

X(1)= d1*Z(1)+d2;
X(2)= d1*Z(2)+d2;

Y(1)= d3*Z(1)+d4;
Y(2)= d3*Z(2)+d4;

end