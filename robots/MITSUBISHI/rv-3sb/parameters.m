%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Mitsubishi RV-3SB robot with 6DOF.
%
%   Author: Héctor Rubén Carías Juárez
%           Universidad Don Bosco de El Salvador
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
function robot = parameters()

robot.name='Mitsubishi RV-3SB 6DOF robotic arm';

%Path where everything is stored for this robot
%robot.path = 'robots/mitsubishi/rv-3sb';

%kinematic data
robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)]';
robot.DH.d='[0.350 0 0 0.270 0 0.085]';
robot.DH.a='[0.095 0.245 0.135 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';

%number of degrees of freedom
robot.DOF = 6;

%rotational: R, translational: T
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%Jacobian matrix
robot.J=[];


%Function name to compute inverse kinematic
% options(1)=1 elbow up solution
% options(1)=-1 elbow down solution
% options(2)=1 wrist up solution
% options(2)=-1 wrist down solution
robot.inversekinematic_fn = 'inversekinematic_rv_3sb(robot, T)';



%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-90) deg2rad(135); %Axis 2, minimum, maximum
                deg2rad(-20) deg2rad(171); %Axis 3
                deg2rad(-160) deg2rad(160); %Axis 4
                deg2rad(-120) deg2rad(120); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(250); %Axis 1, rad/s
                deg2rad(187); %Axis 2, rad/s
                deg2rad(250); %Axis 3, rad/s
                deg2rad(412); %Axis 4, rad/s
                deg2rad(412); %Axis 5, rad/s
                deg2rad(660)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 5.5; %m/s

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [238 232 208]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.5 1.5 -1.5 1.5 0 1.5];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;