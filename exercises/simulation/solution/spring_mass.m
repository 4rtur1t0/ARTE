%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a spring mass problem using a RK4 method.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function spring_mass()
global m k c f
close all;

%initial and final time
t0 = 0;
tfinal = 20;
%initial position and speed
x0 = 0;
xd0 = 0;
% mass, force and constants
f = 1; %N
c = 1.5; % N/m/s
m = 2; % kg
k = 15; %N/m

%Use Runge-Kutta to solve
[t, x] = runge_kutta(@spring_mass_equation, [x0 xd0]', [t0 tfinal], 0.01);

%plot results
figure, 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'b')
legend('Position (m) RK4', 'Speed (m/s) RK4')

%Use ode45 to solve
[t,x]=ode45(@spring_mass_equation, [t0 tfinal], [x0 xd0]'); 
figure, 
plot(t, x(:,1), 'r'), hold
plot(t, x(:,2), 'b')
legend('Position (m) ODE45', 'Speed (m/s)  ODE45')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a second order differential equation.
% Called from function exerciseB()
%
% In order to solve for d2x/dt^2 + c*dx/dt + k*x(t) = f. We can use:
% x1 = x(t)
% x2 = dx/dt
% thus,
% dx1/dt = x2
% dx2/dt = d2x/dt^2=f-c*dx/dt-k*x
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = spring_mass_equation(t, x)
global m k c f
% We must return the solution of
% [dx1/dt; dx2/dt]
xd(1) = x(2);
xd(2) = (1/m)*(f - c*x(2)-k*x(1));
%assure column vector
xd = xd(:);



