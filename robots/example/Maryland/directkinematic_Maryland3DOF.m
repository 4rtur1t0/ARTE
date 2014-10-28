%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [T, pd, pdd]=directkinematic_Maryland3DOF(robot, q, qd, qdd)
%
%   Direct kinematics for the Delta 3DOF parallel robot
% 
%   Inputs:
%   q: joint positions
%   qd: joint speeds
%   qdd: joint accelerations
%
%   The joint vectors are defined in this way for position, velocity and acceleration:
%   q = [theta1 phi11 phi12 theta2 phi21 phi22 theta3 phi31 phi32]
%   qd =[theta1d phi11d phi12d theta2d phi21d phi22d theta3d phi31d phi32d]
%   qd =[theta1dd phi11dd phi12dd theta2dd phi21dd phi22dd theta3dd phi31dd phi32dd]
%
%   where d means dot and dd double dot.
%
% Returns:
%   T: position and orientation (constant) of the end effector.
%        shows T1 and T2 with the 2 posible solution. T is a 4x8 matrix comprised of 
%        two matrices that define the position and orientation of the end effector. The second
%        solution is generally physically unreachable.
%   pd: velocity of the end effector.
%   pdd: acceleration of the end effector.
%
%   pd is computed when qd is provided.
%   pdd is computed whenever qd and qdd are provided
%
%   
%   The direct kinematic problem is computed base only on the active joints
%   and 
%   The 9-dimensional vectors are defined in this way to simplify the calls
%   to the inverse and direct kinematic functions. For example:
%
% T=eye(4);
% T(1,4)=0;
% T(2,4)=0;
% T(3,4)=0.6;
% 
% q=inversekinematic(robot, T);
% drawrobot3d(robot, q(:,8))
% 
% %define position as computed by the inverse kinematic function
% th1=q(1,8);
% th2=q(4,8);
% th3=q(7,8);
% 
% %Joint speeds
% th1d=0;
% th2d=0;
% th3d=0;
% 
% %Joint accelerations
% th1dd=0;
% th2dd=0;
% th3dd=0;
% 
% Next, we solve it for speed and acceleration
% 
% [T, pxyzd, pxyzdd]=directkinematic_Maryland3DOF(robot, [th1 0 0 th2 0 0 th3 0 0], [th1d 0 0 th2d 0 0 th3d 0 0], [th1dd 0 0 th2dd 0 0 th3dd 0 0]);
%
%
% Next, we explain the solution to the direct kinematic problem:
%
%  The constrain equations here for clarity
%   Gamma1=(px-x1)^2+(py-y1)^2+(pz-z1)^2-L2^2;
%   Gamma2=(px-x2)^2+(py-y2)^2+(pz-z2)^2-L2^2;
%   Gamma3=(px-x3)^2+(py-y3)^2+(pz-z3)^2-L2^2;
%  Now expand Gamma1, Gamma2 and Gamma3
%   Gamma1=px^2 + py^2 + pz^2 + x1^2 + y1^2 + z1^2-2(px*x1+py*y1+pz*z1)-L2^2;
%   Gamma2=px^2 + py^2 + pz^2 + x2^2 + y2^2 + z2^2-2(px*x2+py*y2+pz*z2)-L2^2;
%   Gamma3=px^2 + py^2 + pz^2 + x3^2 + y3^2 + z3^2-2(px*x3+py*y3+pz*z3)-L2^2;
%
%   Next, we get rid of the terms px^2, py^2 and pz^2 compute Gamma1-Gamma2,
%   Gamma1-Gamma3 and Gamma2-Gamma3:
%   G12=x1^2+y1^2+z1^2-(x2^2+y2^2+z2^2)-2(px*(x1-x2)+py*(y1-y2)+pz*(z1-z2))
%   G13=x1^2+y1^2+z1^2-(x3^2+y3^2+z3^2)-2(px*(x1-x3)+py*(y1-y3)+pz*(z1-z3))
%   G23=x3^2+y3^2+z3^2-(x4^2+y4^2+z4^2)-2(px*(x2-x3)+py*(y2-y3)+pz*(z2-z3))
%   That can be written as:
%   px*(x1-x2)+py*(y1-y2)+pz*(z1-z2)=(w1-w2)/2;   (A)
%   px*(x1-x3)+py*(y1-y3)+pz*(z1-z3)=(w1-w3)/2;   (B)
%   px*(x2-x3)+py*(y2-y3)+pz*(z2-z3)=(w2-w3)/2;   (C)
% 
%   The last is a system of linear equations in terms of px, py and pz.
%   However, do note that the following makes no sense, since, by
%   construction, det(A)=0, thus, we cannot solve as:
%     A=[x1-x2 y1-y2 z1-z2;
%        x1-x3 y1-y3 z1-z3;
%        x2-x3 y2-y3 z2-z3];
%    w=(1/2)*[x1^2+y1^2+z1^2-x2^2-y2^2-z2^2 x1^2+y1^2+z1^2-x3^2-y3^2-z3^2 ...
%        x2^2+y2^2+z2^2-x3^2-y3^2-z3^2]';
%    
%    p1=inv(A)w;
%
%   Nevertheless, from Equations (A) and (B) we can obtain px of the form
%   px=a1*pz+b1
%   and from Equations (B) and (C) we obtain py of the form:
%   py=a2*pz+b2
%
%   with
%   a1=-((z1-z2)*(y1-y3)-(z1-z3)*(y1-y2))/((x1-x2)*(y1-y3)-(x1-x3)*(y1-y2))
%   b1 = (1/2)*((w1-w2)*(y1-y3)-(w1-w3)*(y1-y2))/((x1-x2)*(y1-y3)-(x1-x3)*(y1-y2))
%   a2=-((z1-z3)*(x2-x3)-(z2-z3)*(x1-x3))/((x2-x3)*(y1-y3)-(x1-x3)*(y2-y3))
%   b2 = (1/2)*((w1-w3)*(x2-x3)-(w2-w3)*(x1-x3))/((x2-x3)*(y1-y3)-(x1-x3)*(y2-y3))
%
%   We finally substitute px=a1*pz+b1 and py=a2*pz+b2 in Gamma1, obtaining a
%   second order polynomial in terms of pz of the form:
%   k1*pz^2+k2*pz+k3=0
%
%   with 
%       k1 = (a1^2+a2^2+1);
%       k2 = 2*(a1*(b1-x1) + a2*(b2-y1)-z1);
%       k3 = (b1-x1)^2 + (b2-y1)^2 + z1^2-L2^2;
%
%   thus, two possible solutions for pz exists as:
%
%   pz = (-k2 + sqrt(k2^2-4*k1*k3))/(2*k1) and
%   pz = (-k2 - sqrt(k2^2-4*k1*k3))/(2*k1)    
% 
% 
%   Author: Arturo Gil
%   Date: 18/12/2014
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

function [T, pd, pdd]=directkinematic_Maryland3DOF(robot, q, qd, qdd)

%retrieve the active joints
th1=q(1);
th2=q(4);
th3=q(7);

%Robot parameters
%geometric parameters, just find a shorter name for clarity
rb=robot.rb;
re=robot.re;
L1=robot.L1;
L2=robot.L2;
Phi=robot.Phi;

%compute the displacements xi, yi zi
x1=(L1*cos(th1)+rb-re)*cos(Phi(1));
y1=(L1*cos(th1)+rb-re)*sin(Phi(1));
z1=L1*sin(th1);

x2=(L1*cos(th2)+rb-re)*cos(Phi(2));
y2=(L1*cos(th2)+rb-re)*sin(Phi(2));
z2=L1*sin(th2);

x3=(L1*cos(th3)+rb-re)*cos(Phi(3));
y3=(L1*cos(th3)+rb-re)*sin(Phi(3));
z3=L1*sin(th3);

%auxiliar w variables
w1=x1^2+y1^2+z1^2;
w2=x2^2+y2^2+z2^2;
w3=x3^2+y3^2+z3^2;

%compute the coefficients of px = a1*pz+b1 and py = a2*pz+b2;
a1=-((z1-z2)*(y1-y3)-(z1-z3)*(y1-y2))/((x1-x2)*(y1-y3)-(x1-x3)*(y1-y2));
b1 = (1/2)*((w1-w2)*(y1-y3)-(w1-w3)*(y1-y2))/((x1-x2)*(y1-y3)-(x1-x3)*(y1-y2));
a2=-((z1-z3)*(x2-x3)-(z2-z3)*(x1-x3))/((x2-x3)*(y1-y3)-(x1-x3)*(y2-y3));
b2 = (1/2)*((w1-w3)*(x2-x3)-(w2-w3)*(x1-x3))/((x2-x3)*(y1-y3)-(x1-x3)*(y2-y3));

%finally, compute the coefficients of the second order polynomial
k1 = (a1^2+a2^2+1);
k2 = 2*(a1*(b1-x1) + a2*(b2-y1)-z1);
k3 = (b1-x1)^2 + (b2-y1)^2 + z1^2-L2^2;

%thus, two possible solution for pz exists as
pz = [(-k2 + sqrt(k2^2-4*k1*k3))/(2*k1) (-k2 - sqrt(k2^2-4*k1*k3))/(2*k1)];

%once known, compute now px and py
px = a1*pz+b1;
py = a2*pz+b2;
   
%init both solutions with homogeneous matrices
T1=eye(4);
T2=eye(4);

%T1 stores the normal pz positive solution
T1(1:3,4)=[px(1), py(1), pz(1)]';
T2(1:3,4)=[px(2), py(2), pz(2)]';

%Yes, returning both solutions!
T=[T1 T2];

%if qd is provided, compute end effector's speed
%We only provide speeds for one of the two solutions
% of the direct kinematic in position, the one with positive z
if exist('qd', 'var')    
    %we have different speeds for each of the two possible configurations
    %of the arm, just compute both
    pd1=compute_direct_speed(robot, q, qd, [px(1), py(1), pz(1)]);
    pd2=compute_direct_speed(robot, q, qd, [px(2), py(2), pz(2)]);
    
    %yes, return both
    pd=[pd1 pd2];       
end


%if qdd is provided, compute end effector's acceleration
if exist('qdd', 'var')
    %we have different speeds for each of the two possible configurations
    %of the arm, just compute both
    pdd1=compute_direct_accel(robot, q, qd, qdd, [px(1), py(1), pz(1)], [pd1(1), pd1(2) pd1(3)]);
    pdd2=compute_direct_accel(robot, q, qd, qdd, [px(2), py(2), pz(2)], [pd2(1), pd2(2) pd2(3)]);
    
    %yes, return both
    pdd=[pdd1 pdd2];    
end


%Compute end effectors speed based on q, qd and the point in space px, py, pz
function pd=compute_direct_speed(robot, q, qd, p)
%geometric parameters
rb=robot.rb;
re=robot.re;
L1=robot.L1;
L2=robot.L2;
Phi=robot.Phi;

% Note that px, py, pz have been computed before!!
%
th1=q(1);
th2=q(4);
th3=q(7);

thd1=qd(1);
thd2=qd(4);
thd3=qd(7);

%retrieve the position as px, py, pz
px=p(1);
py=p(2);
pz=p(3);

%compute the displacements xi, yi zi
x1=(L1*cos(th1)+rb-re)*cos(Phi(1));
y1=(L1*cos(th1)+rb-re)*sin(Phi(1));
z1=L1*sin(th1);

x2=(L1*cos(th2)+rb-re)*cos(Phi(2));
y2=(L1*cos(th2)+rb-re)*sin(Phi(2));
z2=L1*sin(th2);

x3=(L1*cos(th3)+rb-re)*cos(Phi(3));
y3=(L1*cos(th3)+rb-re)*sin(Phi(3));
z3=L1*sin(th3);

%Just here to test that the constraint equations Gamma_i are correct
%     Gamma1=(px-x1)^2+(py-y1)^2+(pz-z1)^2-L2^2;
%     Gamma2=(px-x2)^2+(py-y2)^2+(pz-z2)^2-L2^2;
%     Gamma3=(px-x3)^2+(py-y3)^2+(pz-z3)^2-L2^2;

%compute the derivative of the displacements with respect to th1, th2 and
%th3
dx1=-L1*sin(th1)*cos(Phi(1));
dy1=-L1*sin(th1)*sin(Phi(1));
dz1=L1*cos(th1);

dx2=-L1*sin(th2)*cos(Phi(2));
dy2=-L1*sin(th2)*sin(Phi(2));
dz2=L1*cos(th2);

dx3=-L1*sin(th3)*cos(Phi(3));
dy3=-L1*sin(th3)*sin(Phi(3));
dz3=L1*cos(th3);

Jacob_th=[-2*(px-x1)*dx1-2*(py-y1)*dy1-2*(pz-z1)*dz1             0                      0;
    0           -2*(px-x2)*dx2-2*(py-y2)*dy2-2*(pz-z2)*dz2         0;
    0                                         0  -2*(px-x3)*dx3-2*(py-y3)*dy3-2*(pz-z3)*dz3];


Jacob_pxyz=[2*(px-x1) 2*(py-y1) 2*(pz-z1);
    2*(px-x2) 2*(py-y2) 2*(pz-z2);
    2*(px-x3) 2*(py-y3) 2*(pz-z3)];

%Compute end effector's speed
pd = -inv(Jacob_pxyz)*Jacob_th*[thd1 thd2 thd3]';


%Compute end effectors accel based on q, qd, qdd and the point in space px, py, pz
function pdd=compute_direct_accel(robot, q, qd, qdd, p, pd)

%geometric parameters
rb=robot.rb;
re=robot.re;
L1=robot.L1;
L2=robot.L2;
Phi=robot.Phi;

%Note that px, py, pz have been computed before!!
th1=q(1);
th2=q(4);
th3=q(7);

thd1=qd(1);
thd2=qd(4);
thd3=qd(7);

thdd1=qdd(1);
thdd2=qdd(4);
thdd3=qdd(7);

%retrieve the position as px, py, pz
px=p(1);
py=p(2);
pz=p(3);

%compute the displacements xi, yi zi
x1=(L1*cos(th1)+rb-re)*cos(Phi(1));
y1=(L1*cos(th1)+rb-re)*sin(Phi(1));
z1=L1*sin(th1);

x2=(L1*cos(th2)+rb-re)*cos(Phi(2));
y2=(L1*cos(th2)+rb-re)*sin(Phi(2));
z2=L1*sin(th2);

x3=(L1*cos(th3)+rb-re)*cos(Phi(3));
y3=(L1*cos(th3)+rb-re)*sin(Phi(3));
z3=L1*sin(th3);

%     %Just here to test that the constraint equations Gammai are correct
%     Gamma1=(px-x1)^2+(py-y1)^2+(pz-z1)^2-L2^2;
%     Gamma2=(px-x2)^2+(py-y2)^2+(pz-z2)^2-L2^2;
%     Gamma3=(px-x3)^2+(py-y3)^2+(pz-z3)^2-L2^2;

%compute the derivative of the displacements with respect to th1, th2 and
%th3
dx1=-L1*sin(th1)*cos(Phi(1));
dy1=-L1*sin(th1)*sin(Phi(1));
dz1=L1*cos(th1);

ddx1=-L1*cos(th1)*cos(Phi(1));
ddy1=-L1*cos(th1)*sin(Phi(1));
ddz1=-L1*sin(th1);

dx2=-L1*sin(th2)*cos(Phi(2));
dy2=-L1*sin(th2)*sin(Phi(2));
dz2=L1*cos(th2);

ddx2=-L1*cos(th2)*cos(Phi(2));
ddy2=-L1*cos(th2)*sin(Phi(2));
ddz2=-L1*sin(th2);

dx3=-L1*sin(th3)*cos(Phi(3));
dy3=-L1*sin(th3)*sin(Phi(3));
dz3=L1*cos(th3);

ddx3=-L1*cos(th3)*cos(Phi(3));
ddy3=-L1*cos(th3)*sin(Phi(3));
ddz3=-L1*sin(th3);

%OK, the two Jacobians with respect to theta and Xi
Jacob_th=  [-2*(px-x1)*dx1-2*(py-y1)*dy1-2*(pz-z1)*dz1             0                      0;
    0           -2*(px-x2)*dx2-2*(py-y2)*dy2-2*(pz-z2)*dz2         0;
    0                                         0 -2*(px-x3)*dx3 - 2*(py-y3)*dy3-2*(pz-z3)*dz3];

Jacob_pxyz=[2*(px-x1) 2*(py-y1) 2*(pz-z1);
    2*(px-x2) 2*(py-y2) 2*(pz-z2);
    2*(px-x3) 2*(py-y3) 2*(pz-z3)];

%Now compute Hessian for Gamma1
k1=2*(dx1^2+dy1^2+dz1^2)-2*[(px-x1) (py-y1)  (pz-z1)]*[ddx1 ddy1 ddz1]';

H1=[k1      0     0 -2*dx1 -2*dy1 -2*dz1;
    0      0     0    0     0       0;
    0      0     0    0     0       0;
    -2*dx1  0     0    2     0       0;
    -2*dy1  0     0    0     2       0;
    -2*dz1  0     0    0     0       2];

%Now compute Hessian for Gamma2
k2=2*(dx2^2+dy2^2+dz2^2)-2*[(px-x2) (py-y2)  (pz-z2)]*[ddx2 ddy2 ddz2]';

H2=[0      0     0     0     0       0;
    0      k2    0  -2*dx2 -2*dy2 -2*dz2;
    0      0     0     0     0       0;
    0   -2*dx2   0     2     0       0;
    0   -2*dy2   0     0     2       0;
    0   -2*dz2   0     0     0       2];

%Now compute Hessian for Gamma2
k3=2*(dx3^2+dy3^2+dz3^2)-2*[(px-x3) (py-y3)  (pz-z3)]*[ddx3 ddy3 ddz3]';

H3=[0      0     0     0     0        0;
    0      0     0     0     0        0;
    0      0     k3  -2*dx3 -2*dy3 -2*dz3;
    0      0  -2*dx3   2     0       0;
    0      0  -2*dy3   0     2       0;
    0      0  -2*dz3   0     0       2];



% Finally, compute the acceleration of the end effector as:
%Note that we need to compute first the end effector's speed in cartesian
%coordinates pd.
pdd = -inv(Jacob_pxyz)*(Jacob_th*[thdd1 thdd2 thdd3]'+[[thd1 thd2 thd3 pd(1) pd(2) pd(3)]*H1*[thd1 thd2 thd3 pd(1) pd(2) pd(3)]';
    [thd1 thd2 thd3 pd(1) pd(2) pd(3)]*H2*[thd1 thd2 thd3 pd(1) pd(2) pd(3)]';
    [thd1 thd2 thd3 pd(1) pd(2) pd(3)]*H3*[thd1 thd2 thd3 pd(1) pd(2) pd(3)]']);

