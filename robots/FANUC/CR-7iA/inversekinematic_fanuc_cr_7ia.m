%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_FANUC_CR_7IA(robot, T)	
%   Solves the inverse kinematic problem for the FANUC CR 7IA robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=inversekinematic_fanuc_cr_7ia returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   fanuc=load_robot('FANUC', 'CR_7IA');
%   q = [0 0 0 0 0 0];	
%   T = directkinematic(fanuc, q);
%   %Call the inversekinematic for this robot
%   qinv = inversekinematic(robot, T);
%   check that all of them are feasible solutions!
%   and every Ti equals T
%   for i=1:8,
%        Ti = directkinematic(robot, qinv(:,i))
%   end
%	See also DIRECTKINEMATIC.
%
%   Author: Rubén Alcaraz Poblet
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2019, by Rubén Alcaraz Poblet
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
function q = inversekinematic_fanuc_cr_7ia(robot, T)

% Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alfa = eval(robot.DH.alpha);

% T= [nx ox ax Ex;
%     ny oy ay Ey;
%     nz oz az Ez];
Ex=T(1,4);
Ey=T(2,4);
Ez=T(3,4);

% Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Ex Ey Ez]' - d(6)*W;
px = Pm(1);
py = Pm(2);
pz = Pm(3);

% k: linealization constant
l = sqrt(px^2 + py^2);
k(1) = l;   % q1
k(2) = -l;  % q1 + pi

% Compute q1
c1 = px./k;
s1 = py./k;
q1 = atan2(s1,c1);

% q = [q1         q1+pi;   
%      0          0;
%      0          0;
%      0          0;
%      0          0;
%      0          0];

% Compute q2, q3
denom = a(1)^2 - 2*a(1)*k - a(2)^2 + 2*a(2)*a(3) - a(3)^2 + d(1)^2 - 2*d(1)*pz - d(4)^2 + k.^2 + pz^2;
num1 = sqrt(- a(1)^4 + 4*a(1)^3*k + 2*a(1)^2*(a(2)^2 + a(3)^2 - d(1)^2 + 2*d(1)*pz + d(4)^2 - 3*k.^2 - pz^2) - 4*a(1)*k.*(a(2)^2 + a(3)^2 - d(1)^2 + 2*d(1)*pz + d(4)^2 - k.^2 - pz^2) - a(2)^4 + 2*a(2)^2*(a(3)^2 + d(1)^2 - 2*d(1)*pz + d(4)^2 + k.^2 + pz^2) - (a(3)^2 - d(1)^2 + 2*d(1)*pz + d(4)^2 - k.^2 - pz^2).^2) + 2*a(2)*d(4);
num2 = num1 - 4*a(2)*d(4);

q31 = 2*atan(num1./denom);      % Codo arriba
q32 = - 2*atan(num2./denom);    % Codo abajo
q3 = [q31 q32]; % (1) -> q1; (2) -> q1 + pi

s3 = sin(q3);
c3 = cos(q3);
k = [k k];

s2 = (a(1)*(a(2) + a(3)*c3 + d(4)*s3) - a(2)*k + a(3)*(s3*(d(1) - pz) - c3.*k) - d(4)*(c3*(d(1) - pz) + k.*s3))./(a(2)^2 + 2*a(2)*(a(3)*c3 + d(4)*s3) + a(3)^2 + d(4)^2);
c2 = (a(1) - a(2)*s2 - a(3)*c3.*s2 - d(4)*s2.*s3 - k)./(a(3)*s3 - c3*d(4));

for i=1:length(s2)
    if s2(i)==0 && c2(i)==-1
        q2(i) = pi;
    else
        q2(i) = 2*atan(s2(i)/(1+c2(i)));
    end
end

% q = [q1         q1+pi      q1        q1+pi;
%      q2_1(1)    q2_1(2)    q2_2(1)   q2_2(2);
%      q3_1(1)    q3_1(2)    q3_2(1)   q3_2(2);
%      0          0          0         0;
%      0          0          0         0;
%      0          0          0         0];

q = [q1 q1; q2; q3];

% Resolver la muñeca
for i = 1:4
    B = eye(4);
    temp = q(2,i);
    q(2,i) = q(2,i)+pi/2;
    for j = 1:3
        B = B*dh(q(j,i), d(j), a(j), alfa(j));
    end
    q(2,i) = temp;
    B = B\T;
    
    c5 = B(3,3);
    q5(2*i-1:2*i) = [1 -1].*acos(c5);
    
    % Resolver caso específico
    if c5 == 1 || c5 == -1
        %     s5 = 0
        %     s6 = 0;
        %     c6 = 1;
        q6(2*i-1:2*i) = [0 0];
        
        s4 = -B(1,2);
        c4 = B(1,1)*c5;
        if s4==0 && c4==-1
            q_4 = pi;
        else
            q_4 = 2*atan(s4/(1+c4));
        end
        q4(2*i-1:2*i) = [q_4 q_4];
    else
        s5 = sin(q5(2*i-1:2*i));
        
        s6 = B(3,2)./s5;
        c6 = - B(3,1)./s5;
        
        for j=1:length(s6)
            if s6(j)==0 && c6(j)==-1
                q6(2*i-2+j) = pi;
            else
                q6(2*i-2+j) = 2*atan(s6(j)/(1+c6(j)));
            end
        end
        
        s4 = B(2,3)./s5;
        c4 = B(1,3)./s5;
        
        for j=1:length(s4)
            if s4(j)==0 && c4(j)==-1
                q4(2*i-2+j) = pi;
            else
                q4(2*i-2+j) = 2*atan(s4(j)/(1+c4(j)));
            end
        end
    end
end

% Sort answers
q = [q1(1) q1 q1(2) q1(1) q1 q1(2);...
    q2(1) q2(1) q2(2) q2(2) q2(3) q2(3) q2(4) q2(4);...
    q3(1) q3(1) q3(2) q3(2) q3(3) q3(3) q3(4) q3(4);...
    q4; q5; q6];

% q = [q1         q1         q1+pi     q1+pi    q1      q1      q1+pi   q1+pi;   
%      q2_1(1)    q2_1(1)    q2_1(2)   q2_1(2)  q2_2(1) q2_2(1) q2_2(2) q2_2(2);
%      q3_1(1)    q3_1(1)    q3_1(2)   q3_1(2)  q3_2(1) q3_2(1) q3_2(2) q3_2(2);
%      q4         q4+pi      q4        q4+pi    q4      q4+pi   q4      q4+pi;
%      q5(1)      q5(2)      q5(3)     q5(4)    q5(5)   q5(2)   q5(1)   q5(2);
%      q6(1)      q6(2)      q6(3)     q6(4)    q6(5)   q6(6)   q6(7)   q6(8)];
end