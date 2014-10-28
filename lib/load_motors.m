%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   LOAD_MOTORS Loads a matrix containing the mechanical and electrical data
%               of a set of motors from Maxon Motors.
%
%   For example:
% 	robot.motors = LOAD_MOTORS() returns a matrix where each row defines the
%   data of a particular model. The motor catalog can be found under
%   practicals/session3_inverse_dynamics/motor_catalog
% 	 
%
%   Example: 
%   robot =  load_robot('abb', 'IRB140')
%
%   robot.motors =  load_motors()
%   robot.motors(1,:)
%
%   
%	See also LOAD_ROBOT.
%
%   Author: Arturo Gil. Universidad Miguel Hernï¿½ndez de Elche. email:
%   arturo.gil@umh.es date:   02/01/2013
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
function motors = load_motors(indexes)

%these correspond to Maxon, 167132;

global configuration



% R (Ohm): Coil resistance
% L (H)  : Coil inductance.
% Kv (V/rad/s): speed constant
% Kp (N·m/A): torque constant
% Max_current (A): Max allowable current in the coil.
% Max_speed (rad/s): Max allowable speed of the motor

% please note the constans to translate the units to

motors= [];


Inertia=[];
constants = [];
Viscous = [];
Coulomb= [];
for i=1:length(indexes),
    switch indexes(i)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 1
        % MOD: 264443, EC22
        % 50W,  Brushless
        % Vin=32 Volt
        % Peak torque: 411 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 1
            Inertia= [Inertia 4.63/1e7]; %4.09 g·cm2= 4.09/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.997   0.147e-3  1/(746*(pi/30))   12.8/1000   32.1         21400]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 2
        % MOD: 118895, EC40
        % 120W,  Brushless
        % Vin=30 Volt
        % Peak torque: 1340 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 2
            Inertia= [Inertia 85/1e7]; %85 g·cm2= 4.09/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.518   0.32e-3  1/(389*(pi/30))   24.6/1000   57.9         10500]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
            
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 3
        % MOD: 266052, EC-4pole 45
        % 200W,  Brushless
        % Vin=48 Volt
        % Peak torque: 4420 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 3
            Inertia= [Inertia 200/1e7]; %85 g·cm2= 4.09/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.566   0.172e-3  1/(183*(pi/30))   52.2/1000   84.8         8110]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
       
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 4
        % MOD: 283150, EC-4pole 45
        % 300W,  Brushless
        % Vin=48 Volt
        % Peak torque: 7690 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%        
        case 4
           Inertia= [Inertia 368/1e7]; %85 g·cm2= 4.09/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.710   0.677e-3  1/(84*(pi/30))   114/1000   67.6         3570]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 5
        % MOD: 167132, EC60
        % 400W,  Brushless
        % Vin=48 Volt
        % Peak torque: 11800 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%        
        case 5
           Inertia= [Inertia 831/1e7]; %85 g·cm2= 4.09/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.345   0.273e-3  1/(113*(pi/30))   84.9/1000   139         4960]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
            
            
            
        otherwise
            disp('PLEASE SELECT A VALID MOTOR!!')
    end
end
         
 motors.Inertia = Inertia;
 motors.constants = constants;
 motors.Viscous=Viscous;
 motors.Coulomb = Coulomb;
        
% %Actuator rotor inertia
% motors.Inertia=[200e-6 200e-6 200e-6 33e-6 33e-6 33e-6];
%         
% %Speed reductor at each joint
% %motors.G=[300 300 300 300 300 300];
% %Obtained from motor catalog under practicals/inverse_dynamics
% %                        R(Ohm)  L(H)      Kv (V/rad/s):speed constant     Kp (Nm/A):torque constant        Max_current (A) 
% motors.constants=[0.345  0.273e-3       2.3474e-05               84.9e-3                 139; %these correspond to Maxon, 167132;
%     ];
% 
% %Viscous friction referred to the 
% motors.Viscous = [0 0 0 0 0 0];
% 
% %Coulomb friction factors, motor referred
% %Tc+, Tc-
% motors.Coulomb = [0	0;
%             0	0;
%             0	0;
%             0   0;
%             0   0;
%             0   0];
% 
% 
%     end

