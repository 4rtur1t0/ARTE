%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_KUKA_KR5_ARC(robot, T)	
%   Solves the inverse kinematic problem for the KUKA KR5 ARC robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC__KUKA_KR5_ARC returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   robot=load_robot('KUKA', 'KR160_R1570_nanoC');
%   q = [0 0 0 0 0 0];	
%   T = directkinematic(robot, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:8,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%	See also DIRECTKINEMATIC.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
function q = inversekinematic_kuka_kr160_r1570_nanoC(robot, T)

%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);

% %Evaluate the parameters
% theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
L6=abs(d(6));


%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]' + L6*W; 

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
q1=atan2(Pm(2), Pm(1));


%solve for q2
q2_1=solve_for_theta2(robot, [q1 0 0 0 0 0 0], Pm);
%the other possible solution is q1 + pi
q2_2=solve_for_theta2(robot, [q1+pi 0 0 0 0 0 0], Pm);

%solve for q3
q3_1=solve_for_theta3(robot, [q1 0 0 0 0 0 0], Pm);
%solver for q3 for both cases
q3_2=solve_for_theta3(robot, [q1+pi 0 0 0 0 0 0], Pm);



%the next matrix doubles each column. For each two columns, two different
%configurations for theta4, theta5 and theta6 will be computed. These
%configurations are generally referred as wrist up and wrist down solution
q = [q1         q1         q1        q1       q1+pi   q1+pi   q1+pi   q1+pi;   
     q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
     q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0;
     0          0          0         0         0      0       0       0];
%leave only the real part of the solutions
q=real(q);

%Note that in this robot, the joint q3 has a non-simmetrical range. In this
%case, the joint ranges from 60 deg to -219 deg, thus, the typical normalizing
%step is avoided in this angle (the next line is commented). When solving
%for the orientation, the solutions are normalized to the [-pi, pi] range
%only for the theta4, theta5 and theta6 joints.

%normalize q to [-pi, pi]
q(1,:) = normalize(q(1,:));
q(2,:) = normalize(q(2,:));

% solve for the last three joints
% for any of the possible combinations (theta1, theta2, theta3)
for i=1:2:size(q,2),
    qtemp = solucion_muneca_esferica(robot, q(:,i), T, 1,'geometric'); %wrist up
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i)=qtemp;
    
    qtemp = solucion_muneca_esferica(robot, q(:,i), T, -1, 'geometric'); %wrist up
    qtemp(4:6)=normalize(qtemp(4:6));
    q(:,i+1)=qtemp;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%See geometry
L2=abs(a(2));
L3=abs(d(4));
A2 = abs(a(3));

%See geometry of the robot
%compute L4
L4 = sqrt(A2^2 + L3^2);

%The inverse kinematic problem can be solved as in the IRB 140 (for example)

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = real(acos((L2^2+r^2-L4^2)/(2*r*L2)));

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) = pi/2 - beta - gamma; %elbow up
q2(2) = pi/2 - beta + gamma; %elbow down


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);

%See geometry
L2=abs(a(2));
L3=abs(d(4));

A2 = abs(a(3));

%See geometry of the robot
%compute L4
L4 = sqrt(A2^2 + L3^2);

%the angle phi is fixed
phi=acos((A2^2+L4^2-L3^2)/(2*A2*L4));

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = real(acos((L2^2 + L4^2 - r^2)/(2*L2*L4)));

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = phi - eta; 
q3(2) = -2*pi +( phi + eta); %%%?????????????????????????????????????????

function q = solucion_muneca_esferica(robot, q, T, wrist, method)

switch method
    
     %algebraic solution
    case 'algebraic'
        T01=dh(robot, q, 1);
        T12=dh(robot, q, 2);
        T23=dh(robot, q, 3);
        
        Q=inv(T23)*inv(T12)*inv(T01)*T;
        
        %detect the degenerate case when q(5)=0, this leads to zeros
        % in Q13, Q23, Q31 and Q32 and Q33=1
        thresh=1e-12;
        %detect if q(5)==0
        % this happens when cos(q5) in the matrix Q is close to 1
        if abs(Q(3,3)-1)>thresh 
            %normal solution
            if wrist==1 %wrist up
                q(4)=atan2(-Q(2,3),-Q(1,3));        
                q(6)=atan2(-Q(3,2),Q(3,1));            
                %q(5)=atan2(-Q(3,2)/sin(q(6)),Q(3,3));
            else %wrist down
                q(4)=atan2(-Q(2,3),-Q(1,3))-pi;            
                q(6)=atan2(-Q(3,2),Q(3,1))+pi;            
                %q(5)=atan2(-Q(3,2)/sin(q(6)),Q(3,3));
            end
            if abs(cos(q(6)+q(4)))>thresh 
                cq5=(Q(1,1)+Q(2,2))/cos(q(4)+q(6))-1;
            end
            if abs(sin(q(6)+q(4)))>thresh
                cq5=(-Q(1,2)+Q(2,1))/sin(q(4)+q(6))-1;
            end
            if abs(sin(q(6)))>thresh
                sq5=-Q(3,2)/sin(q(6));
            end
            if abs(cos(q(6)))>thresh
                sq5=Q(3,1)/cos(q(6));
            end
            q(5)=atan2(sq5,cq5);
            
        else %degenerate solution, in this case, q4 cannot be determined,
             % so q(4)=0 is assigned
            if wrist==1 %wrist up
                q(4)=0;
                q(5)=0;
                q(6)=atan2(-Q(1,2)+Q(2,1),Q(1,1)+Q(2,2));
            else %wrist down
                q(4)=-pi;
                q(5)=0;
                q(6)=atan2(-Q(1,2)+Q(2,1),Q(1,1)+Q(2,2))+pi;
            end             
           
        end  
 
       %geometric solution 
    case 'geometric' 
        % T is the noa matrix defining the position/orientation of the end
        % effector's reference system
        vx6=T(1:3,1);
        vz5=T(1:3,3); % The vector a z6=T(1:3,3) is coincident with z5
        
        % Obtain the position and orientation of the system 3
        % using the already computed joints q1, q2 and q3
        T01=dh(robot, q, 1);
        T12=dh(robot, q, 2);
        T23=dh(robot, q, 3);
        T03=T01*T12*T23;
         
        vx3=T03(1:3,1);
        vy3=T03(1:3,2);
        vz3=T03(1:3,3);
        
        % find z4 normal to the plane formed by z3 and a
        vz4=cross(vz3, vz5);	% end effector's vector a: T(1:3,3)
        
        % in case of degenerate solution,
        % when vz3 and vz6 are parallel--> then z4=0 0 0, choose q(4)=0 as solution
        if norm(vz4) <= 0.000001
            if wrist == 1 %wrist up
                q(4)=0;
            else
                q(4)=-pi; %wrist down
            end
        else
            %this is the normal and most frequent solution
            cosq4=wrist*dot(vy3,vz4);
            sinq4=wrist*dot(-vx3,vz4);
            q(4)=atan2(sinq4, cosq4);
        end
        %propagate the value of q(4) to compute the system 4
        T34=dh(robot, q, 4);
        T04=T03*T34;
        vx4=T04(1:3,1);
        vy4=T04(1:3,2);
             
        % solve for q5 
        cosq5=dot(-vy4,vz5);
        sinq5=dot(vx4,vz5);
        q(5)=atan2(sinq5, cosq5);
        
        %propagate now q(5) to compute T05
        T45=dh(robot, q, 5);
        T05=T04*T45;
        vx5=T05(1:3,1);
        vy5=T05(1:3,2);
        
        % solve for q6
        cosq6=dot(vx6,vx5);
        sinq6=dot(vx6,vy5);
        q(6)=atan2(sinq6, cosq6);     
        
    
        
    otherwise
        disp('no method specified in solve_spherical_wrist');
end



