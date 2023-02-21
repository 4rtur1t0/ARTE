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

function test_kinematics_irb140

close all

fprintf('\nTHE DEMO PRESENTS THE DIRECT AND INVERSE KINEMATIC PROBLEM')

%load robot parameters. You can try different robots%
robot=load_robot('ABB', 'IRB140'); 
%adjust 3D view as desired
adjust_view(robot)

% reach this!
T =[0.0000   -0.0000    1.0000    0.4000;
   -0.0000    1.0000    0.0000   -0.0000;
   -1.0000   -0.0000    0.0000    0.6000;
   0         0         0    1.0000];

%Call the inversekinematic for this robot. All the possible solutions are
%stored at qinv. At least, one of the possible solutions should match q
qinv = inversekinematic(robot, T)

test_joints(robot, qinv)

test_solutions(robot, qinv, T);

test_movement(robot, qinv, T);


function test_movement(robot, qinv, T)

q0 = [0, 0, 0, 0, 0, 0];
n = 5
for i=1:size(qinv,2)
   qq = []; 
   qi = qinv(:,i);
   for j=1:6
        v = linspace(q0(j), qi(j), n);
        qq = [qq; v];
   end
   animate(robot, qq)   
end



function test_solutions(robot, qinv, T)
n_solutions = 8;

fprintf('\nNOW WE CAN REPRESENT THE DIFFERENT SOLUTIONS TO ACHIEVE THE SAME POSITION AND ORIENTATION\n')
fprintf('\nNote that some solutions may not be feasible since some joints may be out of range.\n')
correct=zeros(1,n_solutions);
%check that all of them are possible solutions!
for i=1:size(qinv,2)
    
    Ti = directkinematic(robot, qinv(:,i)) %Ti is constant for the different solutions    
    
    % Note that all the solutions may not be feasible. Some of the joints may
    % be out of range. You can test this situation with test_joints
    test_joints(robot, qinv(:,i));
        
    %now draw the robot to see the solution
    drawrobot3d(robot, qinv(:,i))
    
    pause(0.5);
    
    k=sum(sum((T-Ti).^2));
    if k < 0.01 % a simple threshold to find differences in the solution
        correct(1,i)= 1;        
    else
        correct(1,i)= 0; %uncorrect solution
        fprintf('\nERROR: One of the solutions seems to be uncorrect. Sum of errors: %f', i, k);
    end
end

fprintf('\n************** RESULTS **************')

%Display a message if any of the solutions is not correct
if sum(correct)==n_solutions
    fprintf('\nTEST 1--> OK: Every solution in qinv yields the same position/orientation T');
else
    fprintf('\nTEST 1--> ERROR: One or more of the solutions seem to be uncorrect.');
end


fprintf('\n************** ****** **************\n')

