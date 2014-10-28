%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [T, vx, ax, phi, phid]=directkinematic_5R(robot, q)   
%
%   Direct kinematics for the 5R planar parallel robot.
%   
%   This script comprises the computation of the position of the end
%   effector T as a function of q. As well, the speed and acceleration 
%   of the end effector can be also computed. The speed of the passive
%   joints phid are also computed.   
%
%   
%   In general, given a value of active joint coordinates q1, q2, 
%   there exists two possible solutions. The position of the end effector P(x,y) is returned as two different
%   homogeneous matrices T= [T1, T2] with two possible positions of P in
%   each. If the robot is placed at a singular point P (direct kinematic 
%   singularity), then both solutions are coincident. If this is the case, 
%   a differential movement
%   
%   In addition, if qd (q dot, speed), or qdd (q dot dot, acceleration)
%   of the joint variables, is given, the vx (speed) and ax (acceleration) 
%   in cartesian coordinates are also computed.
%
%   Given q1 and q2, in this 5R robot, two possible solutions are feasible.
%   In consequence, vx=[vx1, vx3] where each column stores the speed [vx, vy]
%   for each of the possible solutions
%   
%   Please, note that, given a value of active joint coordinates q1, q2,
%   there exists different possible solutions for P()
%
%   Author: Arturo Gil Aparicio, arturo.gil@umh.es
%   Date: 13/09/2013
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

function [T, vx, ax, phi, phid]=directkinematic_5R(robot, q)
%close all

a1=eval(robot.robot1.DH.a);
a2=eval(robot.robot2.DH.a);

%Link lengths
l1=abs(a1(1));
l2=abs(a1(2));
l3=abs(a2(1)); 
l4=abs(a2(2));
L=robot.L;%2.5;


xA=cos(q(1));
yA=sin(q(1));

xB=cos(q(2))+L;
yB=sin(q(2));

%figure,
plot(0,0,'r.'), %hold
plot(xA, yA,'k.')
plot(xB, yB,'k.')
plot_line([0 0 0], [xA yA 0], 'r', 2)
plot_line([L 0 0], [xB yB 0], 'r', 2)
%plot(L, 0,'*g')


%draw circles around xA, yA and xB yB as a first approximation
x=-0.47:0.001:2;
rr=[];
for i=1:length(x),
    r=solve_poly(x(i), xA, yA, l2);    
    rr=[rr r];    
end
plot(x, real(rr(1,:)))
plot(x, real(rr(2,:)))

x=0.5:0.001:3.085;
rr=[];
for i=1:length(x),
    r=solve_poly(x(i), xB, yB, l3); 
    rr=[rr r];    
end
plot(x, real(rr(1,:)), 'b')
plot(x, real(rr(2,:)), 'b')


%compute polynomial in terms of alpha, beta and delta
af=xA-xB;
be=l2^2-l1^2-l3^2+xB^2+yB^2;
de=(yB-yA);

a=(de^2+af^2);
b=af*be-2*xA*de^2-2*yA*af*de;
c=de^2*(xA^2+yA^2)+be^2/4-yA*be*de-l2^2*de^2;

%two different solutions for x
x1=(-b+sqrt(b^2-4*a*c))/(2*a);
x2=(-b-sqrt(b^2-4*a*c))/(2*a);

%find y from polynomial, given x1 and x2
r1=solve_poly(x1, xA, yA, l2);
r2=solve_poly(x2, xA, yA, l2);
r3=solve_poly(x1, xB, yB, l3);
r4=solve_poly(x2, xB, yB, l3);

%find those solutions that comply with both equations
R1=[r1; r2];
R2=[r3; r4];
%to do this, obtain only those roots that are repeated
y=[];
for i=1:length(R1),
   val=is_double_root(R1(i),R2);
   if val==1
       y=[y R1(i)];
   end
end

%Plot error if an unfeasible solution is found
if (~isreal(y))
    disp('ERROR: directkinematic_5R: unfeasible solution');
end

y1=y(1);
y2=y(2);

%return solutions in homogeneous matrices
T1=eye(4);
T2=eye(4);

T1(1,4)=x1;
T1(2,4)=y1;
T2(1,4)=x2;
T2(2,4)=y2;
T=[];
T=[T1 T2];

% avoid warnings for imaginary parts when plottin
% the above message should be triggered when an unfeasible solution is
% found
y1=real(y(1));
y2=real(y(2));
x1=real(x1);
x2=real(x2);

%plot solutions
plot_line([xA yA 0], [x1 y1 0], 'y', 2)
plot_line([xB yB 0], [x1 y1 0], 'y', 2)

plot_line([xA yA 0], [x2 y2 0], 'g', 2)
plot_line([xB yB 0], [x2 y2 0], 'g', 2)


% in this case, q1, q2, qd1, qd2
if length(q)==4
    fprintf('directkinematic_5R:: Computing speeds');
    q1=q(1);
    q2=q(2);
    qd1=q(3);
    qd2=q(4);
    %compute direct kinematic speeds,
    %given joint values q1 and q2 and joint speeds qd1 and qd2,
    %compute the speed
    [vx1, phi1, phid1]=compute_direct_speeds(robot, [q1 q2], [qd1 qd2], T1);
    [vx2, phi2, phid2]=compute_direct_speeds(robot, [q1 q2], [qd1 qd2], T2);
    vx=[vx1, vx2];
    phi=[phi1, phi2]; %position of passive joints
    phid=[phid1, phid2];%speed of passive joints 
end

ax=0;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Build de coefficients of a polynomial as a function of the
%   points xA, yA and the length of the link L.
%   Solve for second order polynomial:
%   ax^2+bx+c=0;
%   
%   as x=(-b +- sqrt(b^2-4*a*c))/(2*a)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r=solve_poly(x, xAB, yAB, L)
a=1;
b=-2*yAB;
c=x^2-2*x*xAB-L^2+xAB^2+yAB^2;

r =[(-b+sqrt(b^2-4*a*c))/(2*a); (-b-sqrt(b^2-4*a*c))/(2*a)];



function yes_no=is_double_root(R1_i,R2)
thres=0.0001;
[j, k, v]=find(abs(R2-R1_i)<thres);

%in case a match has been found
if length(j)>0
        yes_no=1;
else
        yes_no=0;
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



function [vx, phi, phid]=compute_direct_speeds(robot, q, qd, T)
% caution, q1 and q2 are the active coordinates, 
% the compute jacobians requires every of the 4 joint values
% in this order q={q1, phi1, q2, phi2}

q1=q(1);
q2=q(2);
qd1=qd(1);
qd2=qd(2);

%yes, compute the inverse kinematic to find the passive joint values that 
%correspond to the position specified by T. We choose the only solution
% in which q1 and q2 are equal to the given values
qinv=inversekinematic(robot, T);
for i=1:4,
    qq=qinv(:,i);
    %simple threshold are used to find the most similar solution.
    if(abs(qq(1)-q1)<0.05) && (abs(qq(3)-q2)<0.05)
        % yes, we have used the inversekinematic function to compute
        % the phi1 and phi2 angles
        phi1=qq(2);
        phi2=qq(4);
        [JX, Jq, Jphi]=compute_jacobians_5R(robot, [q1 phi1 q2 phi2]);
        
        if det([JX Jphi])==0
            fprintf('\n WARNING: directkinematic_5R: direct kinematic singularity detected');            
        end
        %Now that the Jacobians are known, compute vx and Phid
        [vxphid]=-inv([JX Jphi])*Jq*[qd1; qd2];
        vx=vxphid(1:2);
        phid=vxphid(3:4);
        phi=[phi1, phi2]';
        return;
    end
end



    
   