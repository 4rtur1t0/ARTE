%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB6620.
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

robot.name= 'ABB_IRB6620LX';

%Path where everything is stored for this robot
robot.path = 'robots/abb/IRB6620';

robot.DH.theta= '[0 q(2) q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[q(1) 0 0 0.887 0 0.2]';
robot.DH.a='[0.458 0.975 0.2 0 0 0]';
robot.DH.alpha= '[0 0 pi/2 pi/2 -pi/2 0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_irb6620lx(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['T' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[0 deg2rad(4); %Axis 1, minimum, maximum metros
                deg2rad(-125) deg2rad(125); %Axis 2, minimum, maximum
                deg2rad(-90) deg2rad(160); %Axis 3
                deg2rad(-300) deg2rad(300); %Axis 4: 
                deg2rad(-130) deg2rad(130); %Axis 5
                deg2rad(-300) deg2rad(300)]; %Axis 6: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(100); %Axis 1, rad/s
                deg2rad(90); %Axis 2, rad/s
                deg2rad(90); %Axis 3, rad/s
                deg2rad(150); %Axis 4, rad/s
                deg2rad(120); %Axis 5, rad/s
                deg2rad(190)];%Axis 6, rad/s
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

%base reference system 
%robot.T0 = inv([1 0 0 0;0 0 1 0; 0 -1 0 0;0 0 0 1]);
robot.T0=eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [240 50 10]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-6 6 -6 6 -4 4];
%read graphics files
robot = read_graphics(robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%link masses (kg)
robot.dynamics.masses=[326.982 89.619 80.846 49.318 7.283 1.686];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[-316.874       1.377          -40.242; %(rx, ry, rz) link 1
    -552.473	 -0.253	 219.498; %(rx, ry, rz) link 2
    -120.048 18.679 2.201;  %(rx, ry, rz) link 3
    1 442.422 -12.159;%(rx, ry, rz) link 4
    0 -89.606 -2.094;%(rx, ry, rz) link 5
    1 16.032 -3.930].*1e-3;%(rx, ry, rz) link 6


%Inertia matrices of each link with respect to its D-H reference system.
robot.dynamics.Inertia=[35.5307630619465,76.3147388769721,62.7770872661309,0.131721786807036,-0.637731134920212,-5.87048385153286;
    13.3386364580491,32.4037001118437,36.1957425196398,0.866790010319889,0.00543891938928600,10.8675859220909;
    1.92553868438793,3.35787383463243,3.78943248613387,-0.290429407043168,-0.204483156477234,-0.199291853261792;
    16.1102718353071,6.81439237229636,10.2418395340107,-0.0218202981960000,0.265212695895164,0.287981031562000;
    -0.0138921189843760,0.0373794232394120,-0.0218695382237880,-0.0585029529843760,-0.0564697985415640,-0.0585059539843760;
    0.00509586529186400,0.00462223710140000,0.00938631719046400,-6.42299520000000e-05,0.000112673711360000,4.40698000000000e-06]

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];