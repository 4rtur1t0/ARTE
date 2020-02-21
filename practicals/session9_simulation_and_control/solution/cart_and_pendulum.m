%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve the following exercises:
%   A) Cart and pendulum.
%              Simulate a cart and pendulum system. Then, try to control it
%              using a P controller.
%
% Help: Function prototype
% [y, t] = runge_kutta(f, y0, [t0 tfinal], timestep)
% where f is the function being integrated as dy/dt = f(t, y).
% y0 are the initial conditions
% t0: initial time
% tfinal: final time.
% h: time step for the calculations.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cart_and_pendulum()
close all;

%uncomment to execute each of the exercises
exerciseA()%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a cart and pendulum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exerciseA()
t0 = 0;
tfinal = 20;
x0 = 0;
xd0 = 0.0;
theta0 = pi/8; %0.05;
thetad0 = 0.0;
[t, x] = runge_kutta(@fcart_and_pendulum, [x0 xd0 theta0 thetad0]', [t0 tfinal], 0.01);

animate_pendulum(t, x)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = fcart_and_pendulum(t, x)
% We must return the solution of
% [dx1/dt; dx2/dt]
mc = 5; % kg mass of the cart
mp = 2; % kg mass of the pendulum (centered at the tip)
g = 9.81; %m/s^ 2
L = 1.8; %Length of the pendulum

% get variables with an easy-to-understand name
th = x(3);
thd = x(4);

% control law!
F = control(x);
%F = 0;

num = g*sin(th) + cos(th) * (-F-mp*L*(thd^2)*sin(th))/(mc+mp);
den = L*(4/3 - mp*cos(th)^2/(mc + mp));
thdd = num/den;

num = F + mp*L*(thd^2)*sin(th)-thdd*cos(th);
den = mc+mp;
xdd = num/den;

xd(1) = x(2);
xd(2) = xdd;
xd(3) = x(4);
xd(4) = thdd;
xd = xd(:);



function F = control(x)
% Computing a simple P controller
ref_theta = 0; % theta reference
ref_x = 0; % stay at the origin

% get variables with an easy-to-understand name
pos_x = x(1);
pos_xd = x(2);
th = x(3);
thd = x(4);

error_theta = ref_theta-th;
error_x = ref_x - pos_x;

P1 = -200;
D1 = 30;

P2 = -2;
%D2 = -1.5;

% the control action!
% A control action on theta
F1 = P1*error_theta + D1*thd;

% A control action to control X
F2 = P2*error_x ; % + D2*pos_xd;

F = F1 + F2;

function animate_pendulum(t, x)
global L
% same dimensions
x = x(:, 1:length(t)); 
% animate the system after the simulation
close all
figure
for i=1:length(t)
    % cart position
    posx_cart = x(1,i);
    posy_cart = 0;
    theta = x(3,i);
    % pendulum position 
    posx_pend = L*sin(theta) + posx_cart;
    posy_pend = L*cos(theta);
    plot(posx_cart, posy_cart, 'rs', 'MarkerSize', 30), hold on
    plot(posx_pend, posy_pend, 'bo', 'MarkerSize', 12)
    xlim([-15 15])
    ylim([-5 5])
    line([posx_cart posx_pend], [posy_cart posy_pend])
    pause(0.01);   
    hold off
end
% plot results
figure
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'g')
plot(t, x(3,:), 'b') 
plot(t, x(4,:), 'k')
legend('Position x (m)', 'Speed xd (m/s)', '\theta (rad)', '\thetad (rad/s)')






