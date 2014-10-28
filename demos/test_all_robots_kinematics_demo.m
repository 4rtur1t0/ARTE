% SCRIPT TO TEST THE KINEMATICS OF ALL SERIAL ROBOTS IN THE LIBRARY

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

function test_all_robots_kinematics_demo
%close all
fprintf('\nTHE DEMO LOADS EVERY ROBOT IN THE LIBRARY AND PERFORMS A KINEMATICS TEST')

%global variable that stores the number of solutions of the inverse
%kinematic for the currently loaded robot
global n_solutions robot fp debug configuration n_q_points

%set debug=1 to draw the robot
debug=1;

%total number of robots in the library
n_robots=44;

%test every robot with this number of different joint values q
n_q_points=500;

%store last tested robot in i
i=44
for i=i:n_robots,
    i
    fp=fopen([configuration.libpath '/demos/test_all_robots_results'],'a');
    
    %local function to load one of the robots
    load_a_robot(i);
    
    fprintf(fp, '\n********TESTING ROBOT********: %s', robot.name);
    
    test_robot_kinematics();
    fclose(fp);
end



function test_robot_kinematics
global robot n_solutions fp debug n_q_points

for i=1:n_q_points,
    q=generate_random_q();
    
    q_inv=[];
    
    %Now compute direct kinematics for this position q
    T = directkinematic(robot, q)
    
    %Set to zero if you want to see the robot transparent
    robot.graphical.draw_transparent=0;
    
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
        
        if debug
        %now draw the robot to see the solution
            drawrobot3d(robot, qinv(:,i))
            pause(1);
        end
        
        k=sum(sum((T-Ti).^2));
        if k < 0.01 % a simple threshold to find differences in the solution
            correct(1,i)= 1;
        else
            correct(1,i)= 0; %uncorrect solution
            fprintf('\nERROR: One of the solutions seems to be uncorrect. Sum of errors: %f', i, k);
            fprintf(fp, '\nERROR: One of the solutions seems to be uncorrect. Sum of errors: %f', i, k);
       end
    end
    %Display a message if any of the solutions is not correct
    if sum(correct)==n_solutions
        fprintf('\nOK: Every solution in qinv yields the same position/orientation T');
    else
        fprintf('\nERROR: One of the solutions seems to be uncorrect.');
        fprintf(fp, '\nERROR: One of the solutions seems to be uncorrect.');
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
        fprintf(fp, '\nERROR: Did not find a matching solution for the initial q');
   end
    
end


%LOAD A ROBOT IN THE LIBRARY FROM A INTEGER INDEX
function load_a_robot(index)

global robot n_solutions
%load robot parameters
% you can try different robots
switch index
    %ABB
    case 1
        robot=load_robot('ABB', 'IRB140'); n_solutions = 8;    
    case 2
        robot=load_robot('ABB', 'IRB120'); n_solutions = 8;
    case 3
        robot=load_robot('ABB', 'IRB1600_6_120'); n_solutions = 8;
    case 4
        robot=load_robot('ABB', 'IRB1600_X145_M2004'); n_solutions = 8;
    case 5    
        robot=load_robot('ABB', 'IRB1600ID'); n_solutions = 8;
    
    case 6   
        robot=load_robot('ABB', 'IRB2400'); n_solutions = 8;
    case 7    
        robot=load_robot('ABB', 'IRB4400'); n_solutions = 2;
    case 8   
        robot=load_robot('ABB', 'IRB4600'); n_solutions = 8;
    case 9    
        robot=load_robot('ABB', 'IRB52'); n_solutions = 8;
    case 10    
        robot=load_robot('ABB', 'IRB6620'); n_solutions = 8;
    case 11    
        robot=load_robot('ABB', 'IRB6620LX'); n_solutions = 4;
    case 12    
        robot=load_robot('ABB', 'IRB6650S_125_350'); n_solutions = 8;
    case 13    
        robot=load_robot('ABB', 'IRB6650S_200_300'); n_solutions = 8;
    case 14    
        robot=load_robot('ABB', 'IRB6650S_90_390'); n_solutions = 8;
    case 15    
        robot=load_robot('ABB', 'IRB760'); n_solutions = 1;     
    case 16    
        robot=load_robot('ABB', 'IRB7600_150'); n_solutions = 8;
    case 17    
        robot=load_robot('ABB', 'IRB7600_400_255_m2000'); n_solutions = 8;
    case 18    
        robot=load_robot('ABB', 'IRB7600_500_230'); n_solutions = 8;
    case 19
         robot=load_robot('ABB', 'IRB1600X_140'); n_solutions = 8;

         %ADEPT
    case 20
        robot=load_robot('ADEPT', 'Viper_s1700D'); n_solutions = 8;
        
        %EPSON
    case 21    
        robot=load_robot('EPSON', 'Prosix_C3_A601C'); n_solutions = 8;
        
        %FANUC
    case 22
        robot=load_robot('FANUC', 'LR_MATE_200iC'); n_solutions = 8;

       %KUKA
    case 23
        robot=load_robot('KUKA', 'KR30_jet'); n_solutions = 4;
    case 24       
        robot=load_robot('KUKA', 'KR5_2ARC_HW'); n_solutions = 8;
    case 25
        robot=load_robot('KUKA', 'KR5_arc'); n_solutions = 8;
    case 26
        robot=load_robot('KUKA', 'KR5_sixx_R650'); n_solutions = 8;
    case 27
        robot=load_robot('KUKA', 'KR5_sixx_R850'); n_solutions = 8;
    case 28
        robot=load_robot('KUKA', 'KR6_2'); n_solutions = 8;
    case 29
        robot=load_robot('KUKA', 'KR90_R2700_pro'); n_solutions = 8;
    case 30
        robot=load_robot('KUKA', 'KR90_R3100_EXTRA'); n_solutions = 8;
    case 31
        robot=load_robot('KUKA', 'KR_1000_1300_TITAN'); n_solutions = 8;
    case 32
        robot=load_robot('KUKA', 'KR_16_arc_HW'); n_solutions = 8;
    case 33
        robot=load_robot('KUKA', 'KR_30_L16_2'); n_solutions = 8;
     case 34
        robot=load_robot('KUKA', 'KR5_scara_R350_Z200'); n_solutions = 2;   

  %MITSUBISHI
    case 35
        robot=load_robot('MITSUBISHI', 'PA-10'); n_solutions = 8;%q=[0 0 0 0 0 0];
    case 36
        robot=load_robot('MITSUBISHI', 'rv-6s'); n_solutions = 8;%q=[0 0 0 0 0 0];
  
        %STAUBLI
    case 37
        robot=load_robot('STAUBLI', 'RX160L'); n_solutions = 8; q=[0.1 -0.6 1 0.4 0.5 0.6]
    case 38
        robot=load_robot('STAUBLI', 'RX170BL'); n_solutions = 8; q=[0.1 -0.6 1 0.4 0.5 0.6]

        %UNIMATE
    case 39
        robot=load_robot('UNIMATE', 'puma560'); n_solutions = 8;

        %STANFORD ARM
    case 40
        robot=load_robot('example', 'stanford'); n_solutions = 4;%q=[0 0 0 0 0 0];

       %EXAMPLE ROBOTS
    case 41
        robot=load_robot('example', 'scara'); n_solutions = 2;
    case 42
        disp('ONLY THE POSOTION AND NOT THE ORIENTATION SHOULD BE COMPARED IN THIS ROBOT')
        robot=load_robot('example', '2dofplanar'); n_solutions = 2;

    case 43
        robot=load_robot('example', '3dofplanar'); n_solutions = 2;
    case 44 
        disp('ONLY THE POSOTION AND NOT THE ORIENTATION SHOULD BE COMPARED IN THIS ROBOT')
       
        robot=load_robot('example', '3dofspherical');n_solutions = 2; %just one possible solutions for this case

      %draw the robot
drawrobot3d(robot)

end

%generate a random q considering the angle range
function q=generate_random_q()

global robot

q=ones(1,robot.DOF);

delta = 0.25;
for i=1:robot.DOF,
    %this line poses a problem for angles with a total range greater than
    %2pi
    %delta = robot.maxangle(i,2)-robot.maxangle(i,1);
    %q(i)=delta*rand + robot.maxangle(i,1);
    q(i)=2*delta*rand-delta;
end

q(2)=-0.5;


q=normalize(q);