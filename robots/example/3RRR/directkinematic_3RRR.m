%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3RRR planar parallel robot
% 
%
% DIRECTKINEMATIC_3RRR(robot, Q)
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions for T = [Q1 Q2 Q3 Q4 Q5 Q6) 
% that can be achieved with the joint angles specified by Q, being Q the
% set of active joints in the mechanism.
%
% 
% 
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function T = directkinematic_3RRR(robot, q)

fprintf('\nComputing direct kinematics for the %s robot', robot.name);

%theta1, theta2 and theta3
t1d=q(1);
t2d=q(2);
t3d=q(3);

%xq, yq is the position of the second arm in the mechanism.
xq=robot.robot2.T0(1, 4);
yq=robot.robot2.T0(2, 4);

%xr, yr is the position of the third arm in the mechanism.
xr=robot.robot3.T0(1, 4);
yr=robot.robot3.T0(2, 4);

%set the geometrical parameters.
a=eval(robot.robot1.DH.a);%(1);
a1=a(1);
b1=a(2);
a=eval(robot.robot2.DH.a);
a2=a(1);
b2=a(2);
a=eval(robot.robot3.DH.a);
a3=a(1);
b3=a(2);

%side of the triangle
h=robot.h;

%coefficient vector
c=[];

c(1)=-2*a1*cos(t1d);
c(2)=-2*a1*sin(t1d);
c(3)=a1*a1-b1*b1;
c(4)=-2*xq-2*a2*cos(t2d);
c(5)=2*h;
c(6)=-2*yq-2*a2*sin(t2d);
c(7)=2*h;
c(8)=xq*xq+yq*yq+a2*a2+h*h+2*xq*a2*cos(t2d)+2*yq*a2*sin(t2d)-b2*b2;
c(9)=-2*xq*h-2*a2*h*cos(t2d);
c(10)=-2*yq*h-2*a2*h*sin(t2d);
c(11)=-2*xr-2*a3*cos(t3d);
c(12)=h;
c(13)=-sqrt(3)*h;
c(14)=-2*yr-2*a3*sin(t3d);
c(15)=sqrt(3)*h;
c(16)=h;
c(17)=xr*xr+yr*yr+a3*a3+h*h+2*xr*a3*cos(t3d)+2*yr*a3*sin(t3d)-b3*b3;
c(18)=-xr*h-yr*sqrt(3)*h-a3*cos(t3d)*h-a3*sin(t3d)*sqrt(3)*h;
c(19)=sqrt(3)*h*xr-yr*h+a3*cos(t3d)*sqrt(3)*h-a3*sin(t3d)*h;
c(20)=c(1)-c(4);
c(21)=-c(5);
c(22)=c(2)-c(6);
c(23)=-c(7);
c(24)=c(3)-c(8);
c(25)=-c(9);
c(26)=-c(10);
c(27)=c(1)-c(11);
c(28)=-c(12);
c(29)=-c(13);
c(30)=c(2)-c(14);
c(31)=-c(15);
c(32)=-c(16);
c(33)=c(3)-c(17);
c(34)=-c(18);
c(35)=-c(19);
c(36)=c(21)*c(31);
c(37)=c(21)*c(32)-c(23)*c(28);
c(38)=c(20)*c(31)+c(21)*c(30)-c(22)*c(28);
c(39)=-c(23)*c(29);
c(40)=c(20)*c(32)-c(22)*c(29)-c(23)*c(27);
c(41)=c(20)*c(30)-c(22)*c(27);
c(42)=-c(25)*c(31);
c(43)=c(23)*c(35)-c(26)*c(32);
c(44)=c(23)*c(34)-c(25)*c(32)-c(26)*c(31);
c(45)=c(22)*c(34)-c(24)*c(31)-c(25)*c(30);
c(46)=c(22)*c(35)+c(23)*c(33)-c(24)*c(32)-c(26)*c(30);
c(47)=c(22)*c(33)-c(24)*c(30);
c(48)=c(25)*c(28)-c(21)*c(34);
c(49)=c(26)*c(29);
c(50)=-c(21)*c(35)+c(25)*c(29)+c(26)*c(28);
c(51)=-c(20)*c(34)-c(21)*c(33)+c(24)*c(28)+c(25)*c(27);
c(52)=-c(20)*c(35)+c(24)*c(29)+c(26)*c(27);
c(53)=c(24)*c(27)-c(20)*c(33);
c(54)=c(1)*c(36)*c(42)+c(2)*c(36)*c(48)+c(3)*c(36)*c(36)+c(42)*c(42)+c(48)*c(48);
c(55)=c(1)*(c(36)*c(44)+c(37)*c(42))+c(2)*(c(36)*c(50)+c(37)*c(48))+2*c(3)*c(36)*c(37)+2*c(42)*c(44)+2*c(48)*c(50);
c(56)=c(1)*(c(36)*c(45)+c(38)*c(42))+c(2)*(c(36)*c(51)+c(38)*c(48))+2*c(3)*c(36)*c(38)+2*c(42)*c(45)+2*c(48)*c(51);
c(57)=c(1)*(c(36)*c(43)+c(37)*c(44)+c(39)*c(42))+c(2)*(c(36)*c(49)+c(37)*c(50)+c(39)*c(48))+c(3)*(2*c(36)*c(39)+c(37)*c(37))+2*c(42)*c(43)+c(44)*c(44)+2*c(48)*c(49)+c(50)*c(50);
c(58)=c(1)*(c(36)*c(46)+c(37)*c(45)+c(38)*c(44)+c(40)*c(42))+c(2)*(c(36)*c(52)+c(37)*c(51)+c(38)*c(50)+c(40)*c(48))+c(3)*(2*c(36)*c(40)+2*c(37)*c(38))+2*c(42)*c(46)+2*c(44)*c(45)+2*c(48)*c(52)+2*c(50)*c(51);
c(59)=c(1)*(c(36)*c(47)+c(38)*c(45)+c(41)*c(42))+c(2)*(c(36)*c(53)+c(38)*c(51)+c(41)*c(48))+c(3)*(2*c(36)*c(41)+c(38)*c(38))+2*c(42)*c(47)+c(45)*c(45)+2*c(48)*c(53)+c(51)*c(51);
c(60)=c(1)*(c(37)*c(43)+c(39)*c(44))+c(2)*(c(37)*c(49)+c(39)*c(50))+2*c(3)*c(37)*c(39)+2*c(43)*c(44)+2*c(49)*c(50);
c(61)=c(1)*(c(37)*c(46)+c(38)*c(43)+c(39)*c(45)+c(40)*c(44))+c(2)*(c(37)*c(52)+c(38)*c(49)+c(39)*c(51)+c(40)*c(50))+c(3)*(2*c(37)*c(40)+2*c(38)*c(39))+2*c(43)*c(45)+2*c(44)*c(46)+2*c(49)*c(51)+2*c(50)*c(52);
c(62)=c(1)*(c(37)*c(47)+c(38)*c(46)+c(40)*c(45)+c(41)*c(44))+c(2)*(c(37)*c(53)+c(38)*c(52)+c(40)*c(51)+c(41)*c(50))+c(3)*(2*c(37)*c(41)+2*c(38)*c(40))+2*c(44)*c(47)+2*c(45)*c(46)+2*c(50)*c(53)+2*c(51)*c(52);
c(63)=c(1)*(c(38)*c(47)+c(41)*c(45))+c(2)*(c(38)*c(53)+c(41)*c(51))+2*c(3)*c(38)*c(41)+2*c(45)*c(47)+2*c(51)*c(53);
c(64)=c(1)*c(39)*c(43)+c(2)*c(39)*c(49)+c(3)*c(39)*c(39)+c(43)*c(43)+c(49)*c(49);
c(65)=c(1)*(c(39)*c(46)+c(40)*c(43))+c(2)*(c(39)*c(52)+c(40)*c(49))+2*c(3)*c(39)*c(40)+2*c(43)*c(46)+2*c(49)*c(52);
c(66)=c(1)*(c(39)*c(47)+c(40)*c(46)+c(41)*c(43))+c(2)*(c(39)*c(53)+c(40)*c(52)+c(41)*c(49))+c(3)*(2*c(39)*c(41)+c(40)*c(40))+2*c(43)*c(47)+c(46)*c(46)+2*c(49)*c(53)+c(52)*c(52);
c(67)=c(1)*(c(40)*c(47)+c(41)*c(46))+c(2)*(c(40)*c(53)+c(41)*c(52))+2*c(3)*c(40)*c(41)+2*c(46)*c(47)+2*c(52)*c(53);
c(68)=c(1)*c(41)*c(47)+c(2)*c(41)*c(53)+c(3)*c(41)*c(41)+c(47)*c(47)+c(53)*c(53);


%these are the coefficients of an eight degree polinomial in terms of
%tan(phi/2)
m(1)=c(54)+c(56)+c(59)+c(63)+c(68);
m(2)=2*(c(55)+c(58)+c(62)+c(67));
m(3)=-2*(2*c(54)+c(56)-2*c(57)-2*c(61)-c(63)-2*c(66)-2*c(68));
m(4)=-2*(3*c(55)+c(58)-4*c(60)-c(62)-4*c(65)-3*c(67));
m(5)=2*(3*c(54)-4*c(57)-c(59)+8*c(64)+4*c(66)+3*c(68));
m(6)=2*(3*c(55)-c(58)-4*c(60)-c(62)+4*c(65)+3*c(67));
m(7)=-2*(2*c(54)-c(56)-2*c(57)+2*c(61)+c(63)-2*c(66)-2*c(68));
m(8)=-2*(c(55)-c(58)+c(62)-c(67));
m(9)=c(54)-c(56)+c(59)-c(63)+c(68);

  %phid = findRoots(m);
%phid=roots(m);

%return roots in terms of t=tan(phi/2)
tanphi2=roots(m(end:-1:1));
count=0; %count the number of solutions
phid=[];
for i=1:length(tanphi2),
    %store real solutions for phi
    if(isreal(tanphi2(i)))
        phid=[phid 2*atan(tanphi2(i))];
        count=count+1;
    end
end

if count==0
    disp('ERROR: No valid solutions found');
end

%re
%phi=2*atan(tanphi2(4));
Temp=eye(4);
T=[];
for i=1:length(phid),
    xad=(c(42)*cos(phid(i))*cos(phid(i))+c(43)*sin(phid(i))*sin(phid(i))+c(44)*cos(phid(i))*sin(phid(i))+c(45)*cos(phid(i))+c(46)*sin(phid(i))+c(47))/(c(36)*cos(phid(i))*cos(phid(i))+c(39)*sin(phid(i))*sin(phid(i))+c(37)*cos(phid(i))*sin(phid(i))+c(38)*cos(phid(i))+c(40)*sin(phid(i))+c(41));
    yad=(c(48)*cos(phid(i))*cos(phid(i))+c(49)*sin(phid(i))*sin(phid(i))+c(50)*cos(phid(i))*sin(phid(i))+c(51)*cos(phid(i))+c(52)*sin(phid(i))+c(53))/(c(36)*cos(phid(i))*cos(phid(i))+c(39)*sin(phid(i))*sin(phid(i))+c(37)*cos(phid(i))*sin(phid(i))+c(38)*cos(phid(i))+c(40)*sin(phid(i))+c(41));
    Temp(1,4)=xad;
    Temp(2,4)=yad;
    Temp(1:3,1)=[cos(phid(i)) sin(phid(i)) 0]';
    Temp(1:3,2)=[-sin(phid(i)) cos(phid(i)) 0]';
    T=[T Temp];
end
figure, hold  
%plot the solutions!
for i=1:size(T,2)/4,
    %points of the end effector
    xa=T(1,i*4);
    ya=T(2,i*4);
    xb=xa+cos(phid(i));
    yb=ya+sin(phid(i));
    xc=xa+cos(phid(i)+pi/3);
    yc=ya+sin(phid(i)+pi/3);
    
    %These are the positions of the passive joints
    xd=a1*cos(t1d);
    yd=a1*sin(t1d);
    
    xe=a1*cos(t2d)+xq;
    ye=a1*sin(t2d)+yq;
    xf=a1*cos(t3d)+xr;
    yf=a1*sin(t3d)+yr;
    
    %plot solutions
   % plot_line([0 0 0], [xa ya 0], 'r', 2)
   % plot_line([xq yq 0], [xb yb 0], 'g', 2)
   % plot_line([xr yr 0], [xc yc 0], 'b', 2)
    %draw the end effector
    plot_line([xa ya 0], [xb yb 0], 'k', 2)    
    plot_line([xb yb 0], [xc yc 0], 'k', 2)
    plot_line([xc yc 0], [xa ya 0], 'k', 2)
    
    %draw arms!
    plot_line([0 0 0], [xd yd 0], 'r', 2)    
    plot_line([xd yd 0], [xa ya 0], 'r', 2)
    %second arm
    plot_line([xq yq 0], [xe ye 0], 'g', 2)    
    plot_line([xe ye 0], [xb yb 0], 'g', 2)
    %third arm
    plot_line([xr yr 0], [xf yf 0], 'b', 2)    
    plot_line([xf yf 0], [xc yc 0], 'b', 2)
    
end


function plot_line(p0, p1, color, w)
x0 = p0(1);
y0 = p0(2);
z0 = p0(3);
x1 = p1(1);
y1 = p1(2);
z1 = p1(3);
% Draw a line between p0 and p1
plot3([x0;x1],[y0;y1],[z0;z1], color, 'LineWidth',w);   

