%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Epson C8L.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   09/01/2012
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

robot.name= 'EPSON_C8L';

%Path where everything is stored for this robot
robot.path = 'robots/EPSON/EPSON_C8L';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)]';
robot.DH.d='[0.472 0 0 0.4 0 0.08]';
robot.DH.a='[0.100 0.4 0.03 0 0 0]';
robot.DH.alpha='[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_EPSON_C8L(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-240) deg2rad(240); %Axis 1, minimum, maximum
                deg2rad(-65) deg2rad(158); %Axis 2, minimum, maximum
                deg2rad(-202) deg2rad(61); %Axis 3
                deg2rad(-200) deg2rad(200); %Axis 4: 
                deg2rad(-135) deg2rad(135); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6: 
           

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(294); %Axis 1, rad/s
                deg2rad(300); %Axis 2, rad/s
                deg2rad(360); %Axis 3, rad/s
                deg2rad(450); %Axis 4, rad/s
                deg2rad(450); %Axis 5, rad/s
                deg2rad(720)];%Axis 6, rad/s
            
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

% ------------------------------------------------------------
% 	         Masa de cada elemento (Kg)
% ------------------------------------------------------------
%         m1 = 20;  m2 = 10;  m3 = 10;  
%         m4 = 8;  m5 = 3;  m6 = 1;

% ------------------------------------------------------------
% 	 Matrices de Inercia (Kg-m^2)
% ------------------------------------------------------------
%               r10I_r01 = zeros(3,3);
%               r20I_r02 = zeros(3,3);
%               r30I_r03 = zeros(3,3);
%               r40I_r04 = 0.47*eye(3);
%               r50I_r05 = 0.47*eye(3);
%               r60I_r06 = 0.15*eye(3);

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [225 20 40]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=0;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.5 0.5 -0.5 0.5 0 1];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;