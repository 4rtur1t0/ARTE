
function motorbike_simulation()
global M Rr u1 g km G A Cx
close all;
Rr = 0.3;
M = 200;
g=9.81;
km = 0.0035; % motor constant
u1 = 1;% pilot's control action, can be modified elsewhere
G = 10; % 5
A = 0.55; % m2 
Cx = 0.32; % penetration coefficient of a Honda 250

% initial position and speed
x0 = 0;
v0 = 0;
% total simulation time
tfinal = 10;


[t, x] = runge_kutta(@vehicle_acceleration, [x0 v0]', [0 tfinal], 0.01);
x = x(:, 1:length(t)); 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'g')
legend('Motorbike Position X (m)', 'Motorbike Speed X(2) (m/s) RK4')
vkmh = 3.6*x(2,:);

figure, 
plot(t, vkmh, 'k') 
ylabel('Motorbike speed (km/h)')

[torque, rpm]=torque_speed_curve();
figure, 
plot(rpm, torque)
title('torque-speed curve')
ylabel('Torque N·m')
xlabel('rev/min')

% find answers from the simulation: acceleration from 0-100 for these parameters
indexes = find(vkmh > 100);
if ~isempty(indexes)
    fprintf('\n0-100 km/h acceleration in %f seconds', t(indexes(1)))
end
indexes = find(vkmh > 150);
if ~isempty(indexes)
    fprintf('\n0-150 km/h acceleration in %f seconds', t(indexes(1)))
end
fprintf('\nMax speed (km/h): %f\n', vkmh(end))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve the acceleration of a motorbike considering
% aerodynamic drag.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = vehicle_acceleration(t, x)
global M

% compute motor force uses the current speed to find the Force that moves
% the motorbike
Fm = compute_motor_force(x);
% compute aerodynamic drag force.
Fr = compute_friction_force(x);

% compute Newton's law
xdd = (1/M)*(Fm-Fr);

% We must return the solution of
% [dx1/dt; dx2/dt;]
xd(1) = x(2);
xd(2) = xdd;
% return a column
xd = xd(:);

function Fm = compute_motor_force(x)
global G Rr u1
v = x(2);
w = v*G/Rr;
% w in rad/s, 
taum = current_torque(w, u1);
Fm = G*taum/Rr;


function Fr = compute_friction_force(x)
global A Cx
v = x(2);
Fr = A*Cx*v^2;


function [torque]=current_torque(w, u1)
% convert w (rad/s) to rpm
w = w*30/(pi);

max_torque = 120;
if w < 2000
    torque = max_torque;
elseif (w >= 2000) && (w <= 8000)
    torque = max_torque - 0.0035*(w-2000);
else
    torque = 0;
end
torque = u1*torque;

% curva de par, only for visualization purposes
function [torque, rpm]=torque_speed_curve()
rpm = 0:1:10000;
par_cte = 60;
torque = [];
for i=1:length(rpm)
    omega = rpm(i);
   if omega < 2000
       par = 60;
   elseif (omega >= 2000) && (omega <= 8000)
       par = 60 - 0.0035*(omega-2000);
   else
       par = 0;
   end
   torque = [torque par];
end
