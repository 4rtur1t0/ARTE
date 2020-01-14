%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find solutions for the differential equations:
% EXERCISE A) dx/dt-x*sin(t)^2=0
%             simulate the solution from t=0 to t=5 s with initial conditions
%             x(0)=0 and dx/dt(0)=0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function differential_equations()

%exerciseA()
exerciseB()


function exerciseA()
% initial and final time
t0 = 0;
tfinal = 5;
% initial position and speed
x0 = 1;

% SIMULATE with different delta_time to observe differences in the results
[t_1, x_1] = runge_kutta(@diff_eqA, [x0]', [t0 tfinal], 1);
[t_2, x_2] = runge_kutta(@diff_eqA, [x0]', [t0 tfinal], 0.5);
[t_3, x_3] = runge_kutta(@diff_eqA, [x0]', [t0 tfinal], 0.1);
[t_4, x_4] = runge_kutta(@diff_eqA, [x0]', [t0 tfinal], 0.01);
[t_5, x_5] = runge_kutta(@diff_eqA, [x0]', [t0 tfinal], 0.001);

% plot results
figure, 
plot(t_1, x_1(1,:), 'r'), hold
plot(t_2, x_2(1,:), 'g'), 
plot(t_3, x_3(1,:), 'b'), 
plot(t_4, x_4(1,:), 'c'), 
plot(t_5, x_5(1,:), 'm'), 

legend('x (\delta t= 1)', 'x (\delta t= 0.5)', 'x (\delta t= 0.1)', 'x (\delta t= 0.01)', 'x (\delta t= 0.001)')


function exerciseB()
% initial and final time
t0 = 0;
tfinal = 10;
% initial position and speed
y0 = 0;
yd0 = 0;
ydd0 = 0;

% SIMULATE with different delta_time to observe differences in the results
[t, x] = runge_kutta(@diff_eqB, [y0 yd0 ydd0]', [t0 tfinal], 0.01);


% plot results
figure, 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'g')
plot(t, x(3,:), 'b')

legend('y', '\dot{y}', '\ddot{y}')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a second order differential equation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = diff_eqA(t, x)
xd = x(1)*sin(t)^2;
xd = xd(:);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a second order differential equation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = diff_eqB(t, x)
dy3 = sin(t)-x(3)-x(2)-x(1);
xd = [x(2) x(3) dy3];
xd = xd(:);



