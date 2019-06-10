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
theta0 = pi/4; %initial angle
x0 = 0;
y0 = 0;
vx0 = v*cos(theta0);
vy0 = v*sin(theta0);

[t, y] = runge_kutta(@parabolic, [x0 y0 vx0 vy0]', [t0 tfinal], 0.01);
y = y(:, 1:length(t)); 
plot(t, y(1,:), 'r'), hold
plot(t, y(2,:), 'g')
plot(t, y(3,:), 'b')
plot(t, y(4,:), 'c')
legend('Position X (m) RK4', 'Position Y (m) RK4', 'Speed X (m/s) RK4', 'Speed Y (m/s) RK4')

figure, 
plot(y(1,:), y(2,:), 'r') 
xlabel('X Position (m)')
ylabel('Y Position (m)')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = parabolic(t, y)
% We must return the solution of
% [dx1/dt; dx2/dt; dx3/dt dx4/dt]
g = 9.81; %m/s^ 2. gravity at equator
b = 0.01; % N*s/m a silly friction coefficient
m=10/1000; %mass of the bullet

vx = y(3);
vy = y(4);
theta = atan2(vy, vx);
V = sqrt(vx^2+vy^2);

%forces in both axes
Fx = -b*vx;
Fy = -m*g-b*vy;

xd(1) = y(3);
xd(2) = y(4);
xd(3) = Fx/m;
xd(4) = Fy/m; %-g-V*b*sin(theta);
xd = xd(:);



