%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the following exercises:
%   A) Mobile on a ramp.
%
% Help: Function prototype
% [y, t] = runge_kutta(f, y0, [t0 tfinal], timestep)
% where f is the function being integrated as dy/dt = f(t, y).
% y0 are the initial conditions
% t0: initial time
% tfinal: final time.
% h: time step for the calculations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function inclined_plane()
close all;

%uncomment to execute each of the exercises
exerciseA()%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Integrate a simple time function. dy/dt = 2*t
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exerciseA()
t0 = 0;
tfinal = 10;
[t, x] = runge_kutta(@mobile, [0 0]', [t0 tfinal], 0.01);
x = x(:, 1:length(t)); 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'b')
legend('Position (m) RK4', 'Speed (m/s) RK4')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = mobile(t, x)
% We must return the solution of
% [dx1/dt; dx2/dt]
g=9.81; %m/s^ 2
th = pi/5;
mu = 0.5;

xd(1) = x(2);
xd(2) = g*(sin(th)-mu*cos(th));
xd = xd(:);




