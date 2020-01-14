%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the following exercises:
%   A) A simulation of a DC motor.
%
% Help: Function prototype
% [y, t] = runge_kutta(f, y0, [t0 tfinal], timestep)
% where f is the function being integrated as dy/dt = f(t, y).
% y0 are the initial conditions
% t0: initial time
% tfinal: final time.
% h: time step for the calculations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function simulate_motor_runge_kutta()
close all;
t0 = 0;
tfinal = 15;

[t, y] = runge_kutta(@motor, [0 0 0]', [t0 tfinal], 0.01);
y = y(:, 1:length(t)); 
plot(t, y(1,:), 'r'), hold
plot(t, y(2,:), 'g')
plot(t, y(3,:), 'b')
legend('Position theta (rad)', 'Speed (rad/s)', 'Current (A)')

figure, 
plot(y(1,:), y(2,:), 'r') 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function for Runge-Kutta.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = motor(t, x)
% We must return the solution of
% [dx1/dt; dx2/dt; dx3/dt dx4/dt]
Jm=500*1e-3; %kg*m2 
R=0.566;%(Ohm)  
L=0.172e-3;%(H)  
Kb=1/(183*(pi/30)); % V/rad/s
Kp= 52.2/1000;
Bm = 0.001;
G=1;
tau = 0; % Nm
V = 5; % Volt

thetadd = (1/Jm)*(-Bm*x(2)+Kp*x(3)-tau/G);
currentd = (1/L)*(V-Kb*x(2)-R*x(3));

xd(1) = x(2);
xd(2) = thetadd;
xd(3) = currentd;
xd = xd(:);



