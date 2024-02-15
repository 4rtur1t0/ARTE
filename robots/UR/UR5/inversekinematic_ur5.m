%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_UR5(robot, T)	
%   Solves the inverse kinematic problem for the UR5 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_UR5 returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   >>ur=load_robot('UR', 'UR5');
%   >>q = [0 0 0 0 0 0];	
%   >>T = directkinematic(ur, q);
%
%   %Call the inversekinematic for this robot
%
%   >> qinv = inversekinematic(ur, T);
%
%   check that all of them are feasible solutions!
%   and every Ti equals T
%
%   for i=1:8,
%        Ti = directkinematic(ur, qinv(:,i))
%   end
%   In addition, one of the solutions in qinv should match the original q.
%
%	See also DIRECTKINEMATIC.
%
%   Author: Arturo Gil Aparicio
%           Universitas Miguel Hernandez, SPAIN.
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
function q = inversekinematic_ur5(robot, T)

% action2: compute q1/so that robot may be at any of the pm.
[q1A, q1B] = solve_for_q1(robot, T);
q1B = atan2(sin(q1B), cos(q1B));

% two possible solutions for q2 and q3 given the two possible pm0 and pm1
[q2Awu, q3Awu, z4Awu] = solve_for_q2_q3(robot, T, q1A, +1);
[q2Awd, q3Awd, z4Awd] = solve_for_q2_q3(robot, T, q1A, -1);
[q2Bwu, q3Bwu, z4Bwu] = solve_for_q2_q3(robot, T, q1B, +1);
[q2Bwd, q3Bwd, z4Bwd] = solve_for_q2_q3(robot, T, q1B, -1);

% group 1: q1A (first solution)
% z4 up
% elbow up (1) and down (2)
qgroup1 = [q1A      q1A      ;
           q2Awu(1) q2Awu(2) ;
           q3Awu(1) q3Awu(2) ;
               0      0      ;
               0      0      ;
               0      0      ];
% find the corresponding angles for q4, q5, q6
for i=1:2
    qi = qgroup1(:, i);
    [q4, q5, q6] = solve_for_UR_wrist(robot, T, z4Awu, qi);
    qgroup1(4:6, i) = [q4 q5 q6]';    
end

% group 2: q1A (first solution)
% z4 down
% elbow up (1) and down (2)
qgroup2 = [q1A        q1A       ;
           q2Awd(1)   q2Awd(2)  ;
           q3Awd(1)   q3Awd(2)  ;
               0         0      ;
               0         0      ;
               0         0      ];
% find the corresponding angles for q4, q5, q6
for i=1:2
    qi = qgroup2(:, i);
    [q4, q5, q6] = solve_for_UR_wrist(robot, T, z4Awd, qi);
    qgroup2(4:6, i) = [q4 q5 q6]';    
end

% group 3: q1B (second solution for q1)
% z4 up
% elbow up (1) and down (2)
qgroup3 = [q1B      q1B      ;
           q2Bwu(1) q2Bwu(2) ;
           q3Bwu(1) q3Bwu(2) ;
               0      0      ;
               0      0      ;
               0      0      ];
% find the corresponding angles for q4, q5, q6
for i=1:2
    qi = qgroup3(:, i);
    [q4, q5, q6] = solve_for_UR_wrist(robot, T, z4Bwu, qi);
    qgroup3(4:6, i) = [q4 q5 q6]';    
end

% group 4: q1B (second solution for q1)
% z4 down
% elbow up (1) and down (2)
qgroup4 = [q1B        q1B       ;
           q2Bwd(1)   q2Bwd(2)  ;
           q3Bwd(1)   q3Bwd(2)  ;
               0         0      ;
               0         0      ;
               0         0      ];
% find the corresponding angles for q4, q5, q6
for i=1:2
    qi = qgroup4(:, i);
    [q4, q5, q6] = solve_for_UR_wrist(robot, T, z4Bwd, qi);
    qgroup4(4:6, i) = [q4 q5 q6]';    
end

% arrange all solutions together
q = [qgroup1 qgroup2 qgroup3 qgroup4];
q_filtered = [];
% filter all solutions... if any solution is complex
% remove the whole column
for i=1:size(q, 2)
    qi = q(:, i);
    if isreal(qi)
        q_filtered = [q_filtered qi];
    else
        fprintf('CAUTION: one of the solutions of the inverse kinematics is not reachable');
    end   
end
q = q_filtered;


%
% Solve for q1, given the point pw
%
function [q1A, q1B] = solve_for_q1(robot, T)
%Evaluate the parameters
d = eval(robot.DH.d);
%See geometry at the reference for this robot
L6=d(6);
L4=d(4);
p = T(1:3, 4);
% z5 parallel to z6
z5 = T(1:3, 3);

% Pw: wrist position (false wrist or pseudo wrist)
% however all possible pw lie on a circle around z3
pw = p - L6*z5; 
R = sqrt(pw(1)^2 + pw(2)^2);
alpha = asin(clip(L4/R, 0, 1));
beta = atan2(pw(2), pw(1));
q1A = alpha + beta;
q1B = pi -alpha + beta;

%
%  Given q1, solve for q2 and q3.
%  A solution is obtained for +z4. And a different one for -z4
%
function [q2, q3, z4] =solve_for_q2_q3(robot, T, q1, signo)
%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);
%See geometry at the reference for this robot
L2=a(2);
L3=a(3);
%See geometry at the reference for this robot
L4=d(4);
L5=d(5);
L6=d(6);
p = T(1:3, 4);
% z5 parallel to z6 (aligned)
z5 = T(1:3, 3);
x6 = T(1:3, 1);

% action 2: compute z1 parallel to z2 and parallel to z3
% compute z1, and assign to z3 since they are parallel axes
A01 = dh(robot, [q1 0 0 0 0 0], 1);
z3 = A01(1:3, 3);

% compute +-z4, given that z3 and z5 are both known
% since z3 is perpendicular to z4
% and z5 is also perpendicular to z4
% if norm z4a < threshold, then z5 = z3
% two solutions +-
z4 = cross(z3, z5);
% if norm(z4) == 0 --> z singular condition
% z3 and z5 are parallel
if norm(z4) > 0.001    
    % compute two differen solutions depending on the
    % direction of z4
    z4 = signo*z4/norm(z4);
else
    fprintf('\nCAUTION, SINGULAR CONTIDION')
    fprintf('\nINFINITE SOLUTIONS')
    % try this, since z3 must be perpendicular to z4
    % and, by construction, z5 must be perpendicular to z4
    % in this case, since z3 and z5 are aligned, anything 
    % perpendicular to z5 may be ok, such as x6, wich is perpendicular
    % to z5 by definition
    %z4 = signo*x6;
    z4 = signo*[0 0 1]';
end

% Now, given that z3, z4 and z5 are known, compute pm
% pm is computed for this solution particular configuration of z4
pm0 = p - L6*z5 - L5*z4 - L4*z3;
pm1 = inv(A01)*[pm0; 1];

%check that the point is reachable
R = sqrt(pm1(1)^2 + pm1(2)^2);
if R > L2+L3
   q2 = [NaN NaN];
   q3 = [NaN NaN];
   return
end
alpha = atan2(pm1(2), pm1(1));
% clip to feasible solutions to avoid complex solutions in beta and eta
cbeta = clip((L2^2+R^2-L3^2)/(2*L2*R), -1, 1);
ceta = clip((L2^2+L3^2-R^2)/(2*L2*L3), -1, 1);
beta = acos(cbeta);
eta = acos(ceta);
% two possible soutions for q2
q2up = alpha + beta - pi/2;
q2down = alpha - beta - pi/2;
% two possible solutions for q3
q3up = eta - pi;
q3down = pi - eta;
q2 = [q2up q2down];
q3 = [q3up q3down];

function [q4, q5, q6] = solve_for_UR_wrist(robot, T, z4, q)

z6 = T(1:3, 3);
z5 = z6;
x6 = T(1:3, 1);

% compute q4, since z4 is known and also x3 and y3
A01 = dh(robot, q, 1);
A12 = dh(robot, q, 2);
A23 = dh(robot, q, 3);
A03 = A01*A12*A23;
x3 = A03(1:3, 1);
y3 = A03(1:3, 2);
a = dot(z4, x3);
b = dot(z4, y3);
q4 = atan2(b, a);

% compute q5, since z5 is known and also x4 and y4
A34 = dh(robot, [0 0 0 q4 0 0], 4);
A04 = A03*A34;
x4 = A04(1:3, 1);
y4 = A04(1:3, 2);
a = dot(z5, -y4);
b = dot(z5, x4);
q5 = atan2(b, a);

% compute q6, since q5 is known and also x6
A45 = dh(robot, [0 0 0 0 q5 0], 5);
A05 = A04*A45;
x5 = A05(1:3, 1);
y5 = A05(1:3, 2);
a = dot(x6, -y5);
b = dot(x6, x5);
q6 = atan2(b, a);

function y = clip(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);


