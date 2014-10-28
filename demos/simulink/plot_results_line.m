%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PLOT_RESULTS_line
%  Represents the error at each joint when performing an independent PID
%  control for each joint. Planned represents the joint references for the 
%  control, whereas followed are the actual joint positions simulated.
%
%  See also INVERSEDYNAMIC, FORWARDDYNAMIC
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
close all
figure, plot(reference), title('Joint references to follow a line in space(rad). ')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')

figure, plot(joint_positions), title('Joint positions (controlled) to follow a line in space(rad). ')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')

figure, plot(reference-joint_positions), title('Error during control (reference-joint_positions)(rad). ')
legend('error q_1', 'error q_2', 'error q_3', 'error q_4', 'error q_5', 'error q_6')

drawrobot3d(robot, reference(end,:)),
%figure, 
pp=[];
for i=1:length(reference),
    
    T=directkinematic(robot,reference(i,:)');
    
     pp=[pp T(1:3,4)];
     
end

plot3(pp(1,:),pp(2,:),pp(3,:), 'k', 'LineWidth', 4), %hold

pp=[];
for i=1:length(joint_positions),
    
    T=directkinematic(robot,joint_positions(i,:)');
    
    pp=[pp T(1:3,4)];
     
end

plot3(pp(1,:),pp(2,:),pp(3,:), 'r', 'LineWidth', 4),

