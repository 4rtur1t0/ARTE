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

close all


fprintf('\nTHE DEMO PRESENTS THE DIRECT AND INVERSE KINEMATIC PROBLEM')

%there are eight possible solutions for the 
%inverse kinematic problem for most of these robots
n_solutions = 8;

%Try different configurations
%beware that, depending on the robot's topology
%not all the eight possible solutions will be feasible
%for an antropomorphic 6R robot.
%q=[0 0 0 0 0 0]
%q = [0.5 pi/8 -pi/8 0.1 0.5 0.6]
%q = [-0.1 -0.8 0.8 0.5 1.5 pi]

%q = [pi/4 pi/4 pi/8 pi/8 pi/8 pi/8]
%q = [-0.9*pi 0 0 0 pi/3 0];	
q = [pi/4 pi/4 pi/4 0 0 0];

%load robot parameters
% you can try different robots
%robot=load_robot('ABB', 'IRB140'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB120'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB1600_6_120'); n_solutions = 8;
%*
%robot=load_robot('ABB', 'IRB1600_X145_M2004'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB1600ID'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB2400'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB4400'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB4600'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB52'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB6620'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB6620LX'); n_solutions = 4;
%robot=load_robot('ABB', 'IRB6650S_125_350'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB7600_150'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB7600_400_255_m2000'); n_solutions = 8;
%robot=load_robot('ABB', 'IRB7600_500_230'); n_solutions = 8;
%ADEPT
%robot=load_robot('adept', 'Viper_s1700D'); n_solutions = 8;
%EPSON
%robot=load_robot('epson', 'Prosix_C3_A601C'); n_solutions = 8;
%FANUC
%robot=load_robot('fanuc', 'LR_MATE_200iC'); n_solutions = 8;
%KUKA
%robot=load_robot('kuka', 'KR30_jet'); n_solutions = 8;
%robot=load_robot('kuka', 'KR5_2ARC_HW'); n_solutions = 8;
%robot=load_robot('kuka', 'KR5_arc'); n_solutions = 8;
%robot=load_robot('kuka', 'KR5_sixx_R650'); n_solutions = 8;
%robot=load_robot('kuka', 'KR5_sixx_R850'); n_solutions = 8;
%robot=load_robot('kuka', 'KR6_2'); n_solutions = 8;
%robot=load_robot('kuka', 'KR90_R2700_pro'); n_solutions = 8;
%robot=load_robot('kuka', 'KR90_R3100_EXTRA'); n_solutions = 8;
%robot=load_robot('kuka', 'KR_1000_1300_TITAN'); n_solutions = 8;
%robot=load_robot('kuka', 'KR_16_arc_HW'); n_solutions = 8;
%robot=load_robot('kuka', 'KR_30_L16_2'); n_solutions = 8;
%MITSUBISHI
%robot=load_robot('mitsubishi', 'pa-10'); n_solutions = 8;%q=[0 0 0 0 0 0];
%robot=load_robot('mitsubishi', 'rv-6s'); n_solutions = 8;%q=[0 0 0 0 0 0];
%STANFORD ARM
%robot=load_robot('stanford', ''); n_solutions = 4;%q=[0 0 0 0 0 0];
%STAUBLI
%robot=load_robot('staubli', 'RX160L'); n_solutions = 8; q=[0.1 -0.6 1 0.4 0.5 0.6]
%UNIMATE
%robot=load_robot('unimate', 'puma560'); n_solutions = 8;



%adjust 3D view as desired
adjust_view(robot)

%there are just 2 solutions for these robots
%q = [pi/2 0.2 0.8 pi/4]
%q = [-pi/4 pi/2 0.5 pi]
%robot=load_robot('kuka', 'KR5_scara_R350_Z200'); n_solutions = 2;
%robot=load_robot('example', 'scara'); n_solutions = 2;
%robot=load_robot('example', '2dofplanar'); n_solutions = 2;
%robot=load_robot('example', '3dofplanar'); n_solutions = 2;
%robot=load_robot('example', 'prismatic');n_solutions = 1; %just one possible solutions for this case


%draw the robot
drawrobot3d(robot, q)

%Now compute direct kinematics for this position q
T = directkinematic(robot, q)

%Set to zero if you want to see the robot transparent
robot.graphical.draw_transparent=0;

%Set to one if you want to see the DH axes
%abb.graphical.draw_axes=1;

%Call the inversekinematic for this robot
% all the possible solutions are stored at qinv
% at least, one of the possible solutions should be coincident with q
qinv = inversekinematic(robot, T);


fprintf('\nNOW WE CAN REPRESENT THE DIFFERENT SOLUTIONS TO ACHIEVE THE SAME POSITION AND ORIENTATION\n')
fprintf('\nNot that some solutions may not be feasible. Some joints may be out of range\n')
correct=zeros(1,n_solutions);
%check that all of them are possible solutions!
for i=1:size(qinv,2),
    
    Ti = directkinematic(robot, qinv(:,i)) %Ti is constant for the different solutions
    
    
    % Note that all the solutions may not be feasible. Some of the joints may
    % be out of range. You can test this situation with test_joints
    test_joints(robot, qinv(:,i));
    
    
    %now draw the robot to see the solution
    drawrobot3d(robot, qinv(:,i))
    
    pause(1);
    
    k=sum(sum((T-Ti).^2));
    if k < 0.01 % a simple threshold to find differences in the solution
        correct(1,i)= 1;        
    else
        correct(1,i)= 0; %uncorrect solution
        fprintf('\nERROR: One of the solutions seems to be uncorrect. Sum of errors: %f', i, k);
    end
end
%Display a message if any of the solutions is not correct
if sum(correct)==n_solutions
    fprintf('\nOK: Every solution in qinv yields the same position/orientation T');
else
    fprintf('\nERROR: One or more of the solutions seems to be uncorrect.');
end

%Now, test if any of the solutions in qinv matches q
delta=(repmat(q',[1 n_solutions])-qinv).^2;
%find the solution that matches the initial q
%and corresponds
i=find(sum(delta,1)<0.01);
if ~isempty(i)
    fprintf('\nOK!: Found a matching solution:\n');
    qinv(:,i)
else
    fprintf('\nERROR: Did not find a matching solution for the initial q');
end
