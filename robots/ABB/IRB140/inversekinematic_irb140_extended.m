%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Q = INVERSEKINEMATIC_IRB140(robot, T)	
%   Solves the inverse kinematic problem for the ABB IRB 140 robot
%   where:
%   robot stores the robot parameters.
%   T is an homogeneous transform that specifies the position/orientation
%   of the end effector.
%
%   A call to Q=INVERSEKINEMATIC_IRB140 returns 8 possible solutions, thus,
%   Q is a 6x8 matrix where each column stores 6 feasible joint values.
%
%   
%   Example code:
%
%   >>abb=load_robot('ABB', 'IRB140');
%   >>q = [0 0 0 0 0 0];	
%   >>T = directkinematic(abb, q);
%
%   %Call the inversekinematic for this robot
%
%   >> qinv = inversekinematic(abb, T);
%
%   check that all of them are feasible solutions!
%   and every Ti equals T
%
%   for i=1:8,
%        Ti = directkinematic(abb, qinv(:,i))
%   end
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
%
% This is an extended version of the inverse kinematic of an ABB IRB140
% robot
%
% The solution includes other possible solutions in the joint ranges.
% In addition, if a
function q_total = inversekinematic_irb140_extended(robot, T)

%Evaluate the parameters
d = eval(robot.DH.d);

%See geometry at the reference for this robot
L6=d(6);

%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);

%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);

% Pm: wrist position
Pm = [Px Py Pz]' - L6*W; 

%first joint, two possible solutions admited: 
% if q(1) is a solution, then q(1) + pi is also a solution
q1=atan2(Pm(2), Pm(1));

% alternate solutions for q1, p.e. solution q1 = 0 sould include q1 = +-pi
q1_a = q1 + pi;
q1_a = atan2(sin(q1_a), cos(q1_a));
q1_b = q1 - pi;
q1_b = atan2(sin(q1_b), cos(q1_b));
% q1_c = q1 + 2*pi;
% q1_d = q1 - 2*pi;
%q1s = [q1 q1_a q1_b q1_c q1_d];
q1s = [q1 q1_a q1_b];
q1s = unique(q1s);
% arrange all posible combinatios with q1 and the possible solutions for
% q2 and q3 (elbow up/down)
q = [];
for i=1:length(q1s)
    %solve for q2 and q3
    q2=solve_for_theta2(robot, [q1s(i) 0 0 0 0 0], Pm);
    q3=solve_for_theta3(robot, [q1s(i) 0 0 0 0 0], Pm);
    if isempty(q2) || isempty(q3)
        continue
    end
    v1 = [q1s(i) q2(1) q3(1) 0 0 0]';
    v2 = [q1s(i) q2(2) q3(2) 0 0 0]';
    q = [q v1 v2];    
end
% for i=1:length(q1s)
%     Ti = directkinematic(robot, q(:,i)); 
%     Pmi = Ti(1:3,4) - L6*Ti(1:3,3); 
%     Pm - Pmi
% end


%leave only the real part of the solutions
q=real(q);

%Note that in this robot, the joint q3 has a non-simmetrical range. In this
%case, the joint ranges from 60 deg to -219 deg, thus, the typical normalizing
%step is avoided in this angle (the next line is commented). When solving
%for the orientation, the solutions are normalized to the [-pi, pi] range
%only for the theta4, theta5 and theta6 joints.

q_total = [];
% solve for the last three joints
% for any of the possible combinations (theta1, theta2, theta3)
for i=1:size(q,2)
    % use solve_spherical_wrist2 for the particular orientation
    % of the systems in this ABB robot
    % use either the geometric or algebraic method.
    % the function solve_spherical_wrist2 is used due to the relative
    % orientation of the last three DH reference systems.
    qi = q(:,i);
    %use either one algebraic method or the geometric     
    [qwa, qwb] = solve_spherical_wrist_irb140_local(robot, qi, T);
    qwa = normalize(qwa');
    qw1 = qwa + [2*pi 0 0]';
    qw2 = qwa + [-2*pi 0 0]';
    qw3 = qwa + [0 0 2*pi]';
    qw4 = qwa + [0 0 -2*pi]';
    qw5 = qwa + [2*pi 0 2*pi]';
    qw6 = qwa + [-2*pi 0 -2*pi]';
    qw7 = qwa + [2*pi 0 -2*pi]';
    qw8 = qwa + [-2*pi 0 2*pi]';
    qis = append_sols(qi, [qwa qw1 qw2 qw3 qw4 qw5 qw6 qw7 qw8]);
    
%     for i=1:size(qis,2)
%          qis(:,i)
%          Ti = directkinematic(robot, qis(:,i)); 
%          k=sum(sum((T-Ti).^2));
%         if k > 0.01 
%            'debug'
%         end
%     end
    
    q_total = [q_total qis];
         
    qwb = normalize(qwb');
    qw1 = qwb + [2*pi 0 0]';
    qw2 = qwb + [-2*pi 0 0]';
    qw3 = qwb + [0 0 2*pi]';
    qw4 = qwb + [0 0 -2*pi]';
    qw5 = qwb + [2*pi 0 2*pi]';
    qw6 = qwb + [-2*pi 0 -2*pi]';
    qw7 = qwb + [2*pi 0 -2*pi]';
    qw8 = qwb + [-2*pi 0 2*pi]';
    qis = append_sols(qi, [qwb qw1 qw2 qw3 qw4 qw5 qw6 qw7 qw8]);
%     
%     for i=1:size(qis,2)
%          qis(:,i)
%          Ti = directkinematic(robot, qis(:,i)); 
%          k=sum(sum((T-Ti).^2));
%         if k > 0.01 
%            'debug'
%         end
%     end
    
    q_total = [q_total qis];  
end
%q_total = unique(q_total, 'columns');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
L2=a(2);
L3=d(4);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];
r = sqrt(p1(1)^2 + p1(2)^2);

beta = atan2(-p1(2), p1(1));
gamma = (acos((L2^2+r^2-L3^2)/(2*r*L2)));

if ~isreal(gamma)
    disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
    %gamma = real(gamma);
end

if r > (L2+L3)
    disp('CANNOT FIND SOLUTION FOR THIS CONFIGURATION')
    q2 = [];
    return 
end

% return two possible solutions: elbow up and elbow down
% the order in the solutions here is important and is coordinated with the function
% solve_for_theta3
q2(1) = pi/2 - beta - gamma; % elbow up
if q2(1) < robot.maxangle(2,1) || q2(1) > robot.maxangle(2,2)
    q2(1) = atan2(sin(q2(1)), cos(q2(1)));
end
q2(2) = pi/2 - beta + gamma; % elbow down
if q2(2) < robot.maxangle(2,1) || q2(2) > robot.maxangle(2,2)
    q2(2) = atan2(sin(q2(2)), cos(q2(2)));
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(robot, q, Pm)

%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);

%See geometry
L2=a(2);
L3=d(4);

%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);

eta = (acos((L2^2 + L3^2 - r^2)/(2*L2*L3)));

if ~isreal(eta)
   disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
   %eta = real(eta);
end

if r > (L2+L3)
    disp('CANNOT FIND SOLUTION FOR THIS CONFIGURATION')
    q3 = [];
    return 
end

%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = pi/2 - eta;
if q3(1) < robot.maxangle(3,1) || q3(1) > robot.maxangle(3,2)
    q3(1) = atan2(sin(q3(1)), cos(q3(1)));
end
q3(2) = eta - 3*pi/2;
if q3(2) < robot.maxangle(3,1) || q3(2) > robot.maxangle(3,2)
    q3(2) = atan2(sin(q3(2)), cos(q3(2)));
end



function [qw1, qw2] = solve_spherical_wrist_irb140_local(robot, q, T)

% [   sin(q4)*sin(q6) - cos(q4)*cos(q5)*cos(q6), cos(q6)*sin(q4) + cos(q4)*cos(q5)*sin(q6), -cos(q4)*sin(q5)]
% [ - cos(q4)*sin(q6) - cos(q5)*cos(q6)*sin(q4), cos(q5)*sin(q4)*sin(q6) - cos(q4)*cos(q6), -sin(q4)*sin(q5)]
% [                            -cos(q6)*sin(q5),                           sin(q5)*sin(q6),          cos(q5)]


% degenerate case

% [ -cos(q4 + q6),  sin(q4 + q6), 0,      0]
% [ -sin(q4 + q6), -cos(q4 + q6), 0,      0]
% [             0,             0, 1, 89/200]
% [             0,             0, 0,      1]

%qtemp = zeros(1,6);
T01=dh(robot, q, 1);
T12=dh(robot, q, 2);
T23=dh(robot, q, 3);


Q=inv(T23)*inv(T12)*inv(T01)*T;

thresh = 0.00001;
% estandar
if 1 - abs(Q(3,3)) > thresh
    q5 = acos(Q(3,3));
    q5_= -q5;

    s5 = sign(q5);
    s5_ = sign(q5_);
    q4 = atan2(-s5*Q(2,3), -s5*Q(1,3));
    q4_ = atan2(-s5_*Q(2,3), -s5_*Q(1,3));

    q6 = atan2(s5*Q(3,2), -s5*Q(3,1));
    q6_ = atan2(s5_*Q(3,2), -s5_*Q(3,1));
else    
    q5 = real(acos(Q(3,3)));
    q5_= q5;

    q4 = 0;
    q4_ = pi;

    q6 = atan2(Q(1,2), -Q(2,2));
    q6_ = q6 - pi;   
end
qw1 = [q4, q5, q6];
qw2 = [q4_, q5_, q6_];

function total_q = append_sols(qi, qws)
total_q = [];
for i=1:size(qws, 2)
    qq = qi;
    qq(4:6) = qws(:,i);
   total_q = [total_q qq];
end

function q = normalize(q)
for i=1:length(q)
   q(i)=atan2(sin(q(i)),cos(q(i))); 
end





