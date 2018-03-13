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
[t, y] = runge_kutta(@mobile, [0 0]', [t0 tfinal], 0.01);
y = y(:, 1:length(t)); 
plot(t, y(1,:), 'r'), hold
plot(t, y(2,:), 'b')
legend('Position (m) RK4', 'Speed (m/s) RK4')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = mobile(t, y)
% We must return the solution of
% [dx1/dt; dx2/dt]
g=9.81; %m/s^ 2
th = pi/6; %ángulo de inclinación
mu = 1.3; %coeficiente de rozamiento (de un neumático!)
 
xd(1) = ....;
xd(2) = ....;

xd = xd(:);




