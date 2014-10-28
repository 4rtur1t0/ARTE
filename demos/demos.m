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

%demo file showing the main capabilities of the library
global robot configuration
close all

fprintf('\nROBOT MANUFACTURING AND PROGRAMMING DEMO\n')
manufacturing_demo

spot_welding_demo

fprintf('\nINVERSE AND DIRECT KINEMATICS DEMO\n')
kinematics_demo

fprintf('\nINVERSE DYNAMICS DEMO\n')
inversedynamics_puma560

fprintf('\nFORWARD DYNAMICS DEMO\n')
forwarddynamics_demo

fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

fprintf('\nA MITSUBISHI PA-10 FOLLOWING A LINE IN SPACE\n')
follow_line_pa10

fprintf('\nDIRECT JACOBIAN DEMO\n')
direct_jacobian_demo


fprintf('\nNOW WE CAN PLOT SOME OF THE ROBOTS INCLUDED IN THE LIBRARY\n')
fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

robot=load_robot('kuka', 'KR5_scara_R350_Z200');
drawrobot3d(robot, [0 0 0 0])
fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

robot=load_robot('kuka', 'KR5_sixx_R650');
drawrobot3d(robot, [0 0 0 0 0 0])
fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

robot=load_robot('kuka', 'KR90_R2700_pro');
drawrobot3d(robot, [0 0 0 0 0 0])
fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

fprintf('\nUSE A TEACHING PENDANT TO PROGRAM THE ROBOT IN RAPID\n')
fprintf('\nOTHER ROBOTS (KUKA, FANUC, MITSUBISHI) CAN ALSO BE PROGRAMMED IN RAPID\n')
robot=load_robot('abb', 'IRB140');
teach

fprintf('\nPRESS ANY KEY TO CONTINUE\n')
pause

fprintf('\nRAPID IS TRANSLATED TO A MATLAB PROGRAM\n')
fprintf('\nYOU CAN SIMULATE A PROGRAM STEP BY STEP\n')
edit test_rapid.m


