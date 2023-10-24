% SCRIPT TEST FOR THE KINEMATIC PROBLEM FOR SERIAL ROBOTS

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
% qinv =
% 
%          0         0         0         0    3.1416    3.1416    3.1416    3.1416
%    -0.0412   -0.0412    1.6597    1.6597   -1.5798   -1.5798   -0.3550   -0.3550
%     0.0720    0.0720    3.0696    3.0696   -0.3825   -0.3825   -2.7591   -2.7591
%     0.0000   -3.1416    3.1416   -0.0000    0.0000    3.1416    0.0000   -3.1416
%    -0.0308    0.0308   -1.5539    1.5539   -1.1793    1.1793   -0.0275    0.0275
%    -0.0000    3.1416   -3.1416         0    3.1416   -0.0000    3.1416   -0.0000

function test_kinematics_irb140_extended_path

% this test considers the ability of the inverse kinematics problem
% to generate connected solutions. Id est:

% generate randomly q1 in range of joints


close all

fprintf('\nTHE DEMO PRESENTS THE DIRECT AND INVERSE KINEMATIC PROBLEM')

%load robot parameters. You can try different robots%
robot=load_robot('ABB', 'IRB140'); 
%adjust 3D view as desired
%adjust_view(robot)
errors = 0;
M = 5000;
for i=1:M
    q = randomq(robot);   
    % Test qs ok
    % q = [1.0463   -0.9490   -3.3887    6.9685 -1.3202   -6.5261];
    % q = [-2.5198    0.7720   -3.7730   -4.1749   -0.0035 -3.1357]
    %q = [-3.1339   -1.3700    0.2087   -3.1445   -1.4276   -6.9000]
    T = directkinematic(robot, q);
    % check that we are outside a singularity
    % and avoid it in case it happens
    if abs(q(5)) < 0.01 
        q(5) = q(5) + pi/5
        T = directkinematic(robot, q);
    end
    if  norm(T(1:2,4)) < 0.01
        q(2) = q(2) + pi/5
        q(3) = q(3) + pi/5
        T = directkinematic(robot, q);
    end
    
    % Call the inversekinematic for this robot. All the possible solutions are
    % stored at qinv. At least, one of the possible solutions should match q
    qinv = inversekinematic(robot, T);
    error = check_solutions1(robot, qinv, T);
    if error
        disp('ERROR')  
        errors = errors + 1;
    end
    
    error = check_solutions2(q, qinv);
    if error
        disp('ERROR') 
        errors = errors + 1;   
    end
end
disp('FINISHED')
disp('Errors found')
errors

%
% Generating random q within the joint ranges
%
function qr = randomq(robot)
qr = zeros(1, robot.DOF);
for i=1:robot.DOF    
    qr(i) = robot.maxangle(i, 1) + (robot.maxangle(i, 2)-robot.maxangle(i, 1))*rand;   
end


function error = check_solutions1(robot, qinv, T)
n_solutions = size(qinv,2);
correct = 0;
% check that all of them are possible solutions!
% and yield the same position and orientation
for i=1:n_solutions
    Ti = directkinematic(robot, qinv(:,i));      
%     T - Ti
    k=sum(sum((T-Ti).^2));
    if k < 0.01 % a simple threshold to find differences in the solution
        correct = correct + 1;
    else
        'debug'
    end
end

%Display a message if any of the solutions is not correct
if correct==n_solutions
    error = 0;
else
    error = 1;
end




function error = check_solutions2(q, qinv)
n_solutions = size(qinv, 2);
%Now, test if any of the solutions in qinv matches q
%find the solution that matches the initial q
%delta is just a squared sum of errors at each of the columns of the matrix
%which store the different solutions of qinv
error = 0;
delta=(repmat(q',[1 n_solutions])-qinv).^2;
i=find(sum(delta,1) < 0.01);
if isempty(i)
    fprintf('\nTEST 2--> ERROR: Did not find a matching solution for the initial q.');
    error = 1;
end



