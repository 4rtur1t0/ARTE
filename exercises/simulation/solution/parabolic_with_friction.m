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
global g k m
close all;
g = 9.81; %m/s^ 2. gravity at equator
% datos de ajustados a 1000 m de https://en.wikipedia.org/wiki/Ballistic_table#/media/File:Ballistic_table_for_7.62x51_mm_NATO_(mil_and_moa).png
% 7.62x51 mm NATO ammunition
theta0 = 0; %initial angle
k = 0.0000005; % N*s/m friction coefficient
m=9.7/1000; %mass of the bullet kg
v=860; % m/s initial speed

theta0 = 0; %initial angle
k = 0.0000005; % N*s/m friction coefficient
m=0.47; %mass of the bullet kg
v=609; % m/s initial speed
 % 914


t0 = 0;
tfinal = 2.2;
x0 = 0;
y0 = 0;
vx0 = v*cos(theta0);
vy0 = v*sin(theta0);

[t, x] = runge_kutta(@parabolic_friction_square, [x0 y0 vx0 vy0]', [t0 tfinal], 0.01);
x = x(:, 1:length(t)); 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'g')
plot(t, x(3,:), 'b')
plot(t, x(4,:), 'c')
legend('Position X (m) RK4', 'Position Y (m) RK4', 'Speed X (m/s) RK4', 'Speed Y (m/s) RK4')

figure, 
plot(x(1,:), x(2,:), 'r') 
xlabel('X Position (m)')
ylabel('Y Position (m)')

x(2,end)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = parabolic_friction_square(t, x)
global k g m
% We must return the solution of
% [dx1/dt; dx2/dt; dx3/dt dx4/dt]
vx = x(3);
vy = x(4);

%forces in both axes
sq = sqrt(vx^2+vy^2);
xd(1) = vx;
xd(2) = vy;
xd(3) = -(k/m)*vx*sq;
xd(4) = -(k/m)*vy*sq - g;
xd = xd(:);



