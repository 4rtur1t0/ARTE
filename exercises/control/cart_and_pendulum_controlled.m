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
function cart_and_pendulum_controlled()
close all;

%uncomment to execute each of the exercises
exerciseA()%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a cart and pendulum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exerciseA()
global L 
L = 0.8; % m length of the pendulum
t0 = 0;
tfinal = 10;
[t, x] = runge_kutta(@fcart_and_pendulum, [0 0 0.01 0.01]', [t0 tfinal], 0.01);
x = x(:, 1:length(t)); 

% animate the system
close all
figure
for i=1:length(t)
    %cart position
    posx_cart = x(1,i);
    posy_cart = 0;
    
    theta = x(3,i);
    %pendulum position 
    posx_pend = L*sin(theta) + posx_cart;
    posy_pend = L*cos(theta);
    plot(posx_cart, posy_cart, 'rs', 'MarkerSize', 30), hold on
    plot(posx_pend, posy_pend, 'bo', 'MarkerSize', 12)
    xlim([-5 5])
    ylim([-0.9 0.9])
    line([posx_cart posx_pend], [posy_cart posy_pend])
    pause(0.05);   
    hold off
end
% plot results
figure
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'g')
plot(t, x(3,:), 'b') 
plot(t, x(4,:), 'k')
legend('Position x (m)', 'Speed xd (m/s)', '\theta (rad)', '\thetad (rad/s)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a constant acceleration on x axis on a ramp.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = fcart_and_pendulum(t, x)
% We must return the solution of
% [dx1/dt; dx2/dt]
global L
M = 5; % kg mass of the cart
m = 5; % kg mass of the pendulum (centered at the tip)
g = 9.81; %m/s^ 2

% Computing a simple P controller
ref = 0; % theta reference
theta = x(3); 
error = ref-theta;
k = 1000;
% the control action!
F = k*error;
% if F > 20
%     F=20;
% elseif F<-20
%     F=-20
% end

% compute xdd and thetadd to simulate the system
num = F + m*sin(x(3))*(g*cos(x(3)) - L*x(4)^2);
den = M + m*(1-cos(x(3))^2);
xdd = num/den;

num = F + g*(M+m)*tan(x(3))-m*L*x(4)^2*sin(x(3));
den = L*((M+m)/cos(x(3)) - m*cos(x(3)));
thetadd = num/den;

xd(1) = x(2);
xd(2) = xdd;
xd(3) = x(4);
xd(4) = thetadd;
xd = xd(:);






