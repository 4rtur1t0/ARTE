%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ADEPT eCobra 600.
%
% The authors of this script are:
%   Adrian Peidro Vidal
%		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function robot = parameters()

robot.name= 'ADEPT_eCobra_600';

%Path where everything is stored for this robot
robot.path = 'robots/adept/eCobra_600'

robot.DH.theta= '[q(1) q(2) 0 q(4)]';
robot.DH.d='[0.342 0 q(3)+0.165 0]';
robot.DH.a='[0.325 0.275 0 0]';
robot.DH.alpha= '[0 pi 0 0]';

robot.J=[];

% robot.inversekinematic_fn = 'inversekinematics_VIPER850(robot, T)';
robot.inversekinematic_fn = [];
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 4;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'P' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-190) deg2rad(45); %Axis 2, minimum, maximum
                deg2rad(-29) deg2rad(256); %Axis 3
                deg2rad(-190) deg2rad(190); %Axis 4: Unlimited (400? default)
                ]; %Axis 6: Really Unlimited to (800? default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(250); %Axis 1, rad/s
                deg2rad(250); %Axis 2, rad/s
                deg2rad(250); %Axis 3, rad/s
                deg2rad(375); %Axis 4, rad/s
                ];%Axis 6, rad/s
    
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 7.6; %m/s



%base reference system
robot.T0 = eye(4); 



%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;%0.5;
%draw DH systems
robot.graphical.draw_axes=0;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.8 0.8 -0.8 0.8 0 1];
%read graphics files
robot = read_graphics(robot);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DYNAMIC PARAMETERS
%   WARNING! These parameters do not correspond to the actual VIPER 850
%   robot. They have been introduced to demonstrate the necessity of 
%   simulating the robot and should be used only for educational purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[10.0840645443765 7.99763638499999 7.82396742420414 6.79135586560867 2.69519416947469 0.582726761463460 0.0250548498725116];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[  37.34e-3	 -7.59e-3	 291.45e-3;     %(rx, ry, rz) link 1
                       8.82e-3	 158.89e-3	     -116.12e-3; %(rx, ry, rz) link 2
                       45.72e-3      -60.41e-3       6.77e-3;     %(rx, ry, rz) link 3
                        0.65e-3       5.18e-3      304.45e-3;     %(rx, ry, rz) link 4
                        0        -16.20e-3        -0.43e-3;  %(rx, ry, rz) link 5
                        0        0.10e-3     102.45e-3]; %(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[725041607.30e-9     742053235.46e-9   58888719.46e-9    -4612005.75e-9    -20201400.03e-9  100257432.34e-9;
                        473299566.08e-9    122717629.66e-9	   374750563.05e-9	   12544273.08e-9	  -140558617.88e-9	  -8394816.58e-9;
                        73135068.37e-9	    43626156.19e-9	   96299686.97e-9	   -34759300.78e-9	  -1593782.48e-9	 1209250.25e-9;
                        262441613.90e-9     261164434.73e-9    5256188.91e-9	  15725.91e-9	  4591746.99e-9	 471721.51e-9;
                        869212.45e-9        370702.33e-9       939398.99e-9	  4.39e-9	  -4.96e-9	 3.05e-9;
                        280731.23e-9	     284457.82e-9	   6143.16e-9	  0	   217.96e-9	 0];



%robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
%robot.motors.G=[300 300 300 300 300 300];
