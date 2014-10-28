% SCRIPT TEST TO LOAD ALL ROBOTS

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


fprintf('\nTHE DEMO LOADS ALL ROBOTS AND PLOTS THEM')

%there are eight possible solutions for the 

%load robot parameters
% you can try different robots
robot=load_robot('abb', 'IRB52'); 
robot=load_robot('abb', 'IRB140'); 
robot=load_robot('abb', 'IRB1600_6_120');  
robot=load_robot('abb', 'IRB1600_X145_M2004');  
robot=load_robot('abb', 'IRB1600ID');  
robot=load_robot('abb', 'IRB2400');  
%robot=load_robot('abb', 'IRB4400');  
robot=load_robot('abb', 'IRB4600');  
robot=load_robot('abb', 'IRB6620');  
robot=load_robot('abb', 'IRB6620LX'); 
robot=load_robot('abb', 'IRB6650S_125_350');  
robot=load_robot('abb', 'IRB7600_150');  
robot=load_robot('abb', 'IRB7600_400_255_m2000');  
robot=load_robot('abb', 'IRB7600_500_230');  
%ADEPT
robot=load_robot('adept', 'Viper_s1700D');  
%EPSON
robot=load_robot('epson', 'Prosix_C3_A601C');  
%FANUC
robot=load_robot('fanuc', 'LR_MATE_200iC');  
%KUKA
robot=load_robot('kuka', 'KR5_2ARC_HW');  
robot=load_robot('kuka', 'KR5_arc');  
robot=load_robot('kuka', 'KR5_scara_R350_Z200');  
robot=load_robot('kuka', 'KR5_sixx_R650');  
robot=load_robot('kuka', 'KR5_sixx_R850');  
robot=load_robot('kuka', 'KR6_2');  
robot=load_robot('kuka', 'KR30_jet');  
robot=load_robot('kuka', 'KR90_R2700_pro');  
robot=load_robot('kuka', 'KR90_R3100_EXTRA');  
robot=load_robot('kuka', 'KR_16_arc_HW');  
robot=load_robot('kuka', 'KR_30_L16_2');  
robot=load_robot('kuka', 'KR_1000_1300_TITAN');  
%MITSUBISHI
robot=load_robot('mitsubishi', 'pa-10');  
robot=load_robot('mitsubishi', 'rv-6s');  
%STANFORD ARM
robot=load_robot('example', 'stanford'); 
%STAUBLI
robot=load_robot('staubli', 'RX160L');   
%UNIMATE
robot=load_robot('unimate', 'puma560');  


%EXAMPLE ROBOTS
robot=load_robot('example', '2dofplanar'); 
robot=load_robot('example', '3dofplanar'); 
robot=load_robot('example', 'scara'); 
robot=load_robot('example', 'stanford'); 



