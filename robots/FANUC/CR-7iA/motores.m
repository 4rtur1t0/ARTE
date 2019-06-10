function motors = motores(indices)

%these correspond to MAXON;

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
for i=1:length(indices)
    switch indices(i)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 1
        % MOD: 136196
        % 150W,  Brushless
        % Vin = 12 Volt
        % Nominal torque: 184 mN·m
        % Peak torque: 872 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 1
            Inertia= [Inertia 119/1e7]; %119 g·cm2 = 119/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.275   0.0797e-3  1/(478*(pi/30))     20       9.53         4380]; 
    
            Viscous = [Viscous 0];
                              %C+ C-                      
            Coulomb = [Coulomb; 0 0];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   INDEX 2
        % MOD: 136202
        % 150W,  Brushless
        % Vin = 12 Volt
        % Nominal torque: 167 mN·m
        % Peak torque: 1380 mN·m
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 2
            Inertia= [Inertia 119/1e7]; %119 g·cm2 = 119/10^7 Kg·m^2 
         
            constants = [constants;  
      % R(Ohm)  L(H)             Kv            Kp       Max_current  Nominal_speed
        0.101   0.0266e-3  1/(827*(pi/30))     11.5       15.6         8470]; 
    
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
end
