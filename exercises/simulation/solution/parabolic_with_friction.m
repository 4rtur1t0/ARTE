%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the following exercises:
%   A) Parabolic. with friction.
%
% Help: Function prototype
% [y, t] = runge_kutta(f, y0, [t0 tfinal], timestep)
% where f is the function being integrated as dy/dt = f(t, y).
% y0 are the initial conditions
% t0: initial time
% tfinal: final time.
% h: time step for the calculations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function parabolic_with_friction()
close all;
t0 = 0;
tfinal = 150;
v=800; % m/s initial speed
theta0 = 45; %initial angle
x = 0;
y = 0;
vx = v*cos(theta0);
vy = v*sin(theta0);

[t, y] = runge_kutta(@parabolic, [x y vx vy]', [t0 tfinal], 0.01);
y = y(:, 1:length(t)); 
plot(t, y(1,:), 'r'), hold
plot(t, y(2,:), 'g')
plot(t, y(3,:), 'b')
plot(t, y(4,:), 'c')
legend('Position X (m) RK4', 'Position Y (m) RK4', 'Speed X (m/s) RK4', 'Speed Y (m/s) RK4')

figure, 
plot(y(1,:), y(2,:), 'r') 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = parabolic(t, y)
% We must return the solution of
% [dx1/dt; dx2/dt; dx3/dt dx4/dt]
g = 9.81; %m/s^ 2. gravity at equator
b = 0.01; % N*s/m a silly friction coefficient

vx = y(3);
vy = y(4);
theta = atan2(vy, vx);
V = sqrt(vx^2+vy^2);

xd(1) = y(3);
xd(2) = y(4);
xd(3) = -V*b*cos(theta);
xd(4) = -g-V*b*sin(theta);
xd = xd(:);



