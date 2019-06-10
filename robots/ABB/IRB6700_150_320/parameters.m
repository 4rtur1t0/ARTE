%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB6700_150_320.
%
%   Author: Carlos Hernández. Universidad Miguel Hernández de Elche. 
%   email: ************ date:   24/05/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

robot.name= 'ABB_IRB6700_150_320';

robot.DH.theta= '[q(1) pi/2+q(2) q(3) q(4) pi+q(5) q(6)]';
robot.DH.d='[0.780 0 0 1.5925 0 0.200]';
robot.DH.a='[0.377 1.280 0.200 0 0 0]';
robot.DH.alpha= '[pi/2 0 pi/2 -pi/2 -pi/2 0]';

robot.J=[];

robot.inversekinematic_fn = 'inversekinematic_6700_150_320(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-65) deg2rad(85); %Axis 2, minimum, maximum
                deg2rad(-180) deg2rad(70); %Axis 3
                deg2rad(-300) deg2rad(300); %Axis 4: 
                deg2rad(-130) deg2rad(130); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(100); %Axis 1, rad/s
                deg2rad(90); %Axis 2, rad/s
                deg2rad(90); %Axis 3, rad/s
                deg2rad(170); %Axis 4, rad/s
                deg2rad(120); %Axis 5, rad/s
                deg2rad(190)];%Axis 6, rad/s

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            % end effectors maximum velocity
robot.linear_velmax = 1.6; %m/s, unavailable from datasheet

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
robot.graphical.draw_transparent=1;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-3 3 -3 3 0 4];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[226.9 240.3 179.9 226.9 24.4 2.5];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-0.162       -0.163         -0.025; %(rx, ry, rz) link 1
                      -0.668       -0.045          0.273; %(rx, ry, rz) link 2
                      -0.084       -0.005         -0.055; %(rx, ry, rz) link 3
                       0            0.693          0.010; %(rx, ry, rz) link 4
                       0.002        0.003          0.016; %(rx, ry, rz) link 5
                       0            0             -0.019];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[17.523      21.55       29.966      10.934	0.300	-0.918;
                        21.721      155.632     139.179 	8.200	-2.966	-41.153;
                        8.375       7.546       8.736       -1.268	1.182	0.034;
                        179.088     2.601       177.904 	-0.001	0.167   0.001;
                        0.265       0.248       0.101       0.002	-0.005	0.004;
                        0.008       0.008       0.014       0.000	0.000	0.000];



robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[100 100 50 300 300 50];
