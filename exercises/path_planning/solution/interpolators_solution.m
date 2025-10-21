%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	EXERCISES ON INTERPOLATORS .
%   - Exercise 1: Dealing with a first order planner.
%   - Exercise 2: Derivation of a second order planner.
%   - Exercise 3: A more complex planning using first order interpolators.
%   - Exercise 4: Derive a trapezoidal acceleration profile using a 4th
%   order interpolator.
%   - Exercise 5: Animate a 6 DOF robot with the joint trajectory computed
%   in exercise 4.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2016, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function interpolators_solution
close all
% exercise1()
% exercise2()
% exercise3()
exercise4();
%exercise5()
%exercise6()


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 1:
%   a) Plan a joint trajectory from q1 to q2 using a linear interpolator.
%   The movement must be performed at a constant speed of 2 rad/s.
%   
%   b) Complete the function  first_order in order to return the required
%   data.
%   
%   b) Plot the trajectory in position with a delta of 0.001 seconds. Plot
%   the speed of the trajectory. Compute the accelerations at every time step.
%   
%   c) Present your conclusions. Which aspects could/should be improved?
%           Acceleration tends to infinite values at the starting and end
%           of the trajectory, due to a sudden change in speed. This
%           implies that unnecessary torques will be required
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise1()
% Go from q1=0 to q2=0.5 rad
q=[0 1.8];
%At a constant speed.
qd=2; %rad/s
% DEFINE t = [T1 T2]
T1 = 0;
T2 = (q(2)-q(1))/qd;
t=[T1 T2];
%the minimum difference in seconds between times to compute q(t) functions
delta_t=0.001;

%FIRST ORDER PLANNER FROM q1 to q2 
figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('FIRST ORDER PLANNER. EXERCISE 1'), hold on
[q_t, qd_t, time, k]=first_order([q(1) q(2)], [t(1) t(2)], delta_t); 
plot(time, q_t, 'r'), plot(time, qd_t, 'g')

legend('Position (rad)','Speed (rad/s)')
%WRITE EQUATION
disp('The equation computed can be written as:')
fprintf('\n q(t)=k1+k2*t=%.3f+%.3f*t', k(1), k(2))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 2:
%   a) Derive a method to compute a second order polynomial of the form
%       q(t) = k1 + k2*t + k3*t^2
%       write down the equations needed.
%   b) Program a method named of the form:
%
%   [q_t, qd_t, qdd_t, t, k]=second_order(q, qd, t, delta_t)
%
%   with:
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       qd: the initial speed at t1.
%       t: a time vector with t = [t1 t2]
%       delta_t: the sample time between computed q(t)'s
%
%   Returns:
%       q: the values of q(t) as a function of the time vector used.
%       qd: the speed qd(t) as a function of the time vector (which is a constant in this case)
%       t: the time vector used.
%       k: the polynomial coefficients k that allow to compute q(t) as:
%               q(t) = k(1) + k(2)*t + k(3)*t^2
%
%   c) Present your conclusions. Which aspects could/should be improved?
%           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise2()
% Go from q1=0 to q2=1.5 rad
q=[0 2.5];
qd = 0.3; % rad/s (CHANGE FROM 0 TO 2.3)
%Time vector
t=[0 12];
%the minimum difference in seconds between times to compute q(t) functions
delta_t=0.001;

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s), q_{dd} (rad/s/s)'), title('SECOND ORDER PLANNER'), hold on
[q_t, qd_t, qdd_t, time, k]=second_order([q(1) q(2)], qd(1),[t(1) t(2)], delta_t);
plot(time, q_t, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
legend('Position (rad)','Speed (rad/s)', 'Acceleration (rad/s/s)')

%WRITE EQUATION
disp('The equation computed can be written as:')
fprintf('\n q(t)=k1+k2*t+k3*t^2=%.3f+%.3f*t+%.3f*t^2', k(1), k(2), k(3))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 3:
%   a) Use a first order planner to perform the following movement on a
%   single joint of a robot.
%       q1 = 0.2 rad. t1 = 0 s
%       q2 = 0.5 rad. t2 = 6 s
%       q3 = 0.8 rad. t3 = 8 s
%       q4 = 1.5 rad. t4 = 10 s
%   b) Plot the whole trajectory in speed and position.
%   c) Present your main conclusions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise3()
% Go from q1=0 to q2=0.5 rad
%q=[q1 q2 q3 q4];
q=[0.2 0.5 0.8 1.5];
%t=[t1 t2 t3 t4];
t=[0 6 8 10];

%the minimum difference in seconds between times to compute q(t) functions
delta_t=0.001;

figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s)'), title('FIRST ORDER PLANNER. EXERCISE 3'), hold on
[q_t, qd_t, time, k]=first_order([q(1) q(2)],[t(1) t(2)], delta_t); plot(time, q_t, 'r'), plot(time, qd_t, 'g')
[q_t, qd_t, time, k]=first_order([q(2) q(3)],[t(2) t(3)], delta_t); plot(time, q_t, 'r'), plot(time, qd_t, 'g')
[q_t, qd_t, time, k]=first_order([q(3) q(4)],[t(3) t(4)], delta_t); plot(time, q_t, 'r'), plot(time, qd_t, 'g')
legend('Joint position q (rad)', 'Joint speed q (rad/s)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 4:
%   fifth order interpolator.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise4()
  
% Go from q1=0 to q2=1.5 rad
% q=[0 2.5];
% qd = [ 0 0]; % rad/s (CHANGE FROM 0 TO 2.3)
% qdd = [0 0]
q=[0 pi/2];
qd = [0 0]; % rad/s (CHANGE FROM 0 TO 2.3)
qdd = [0 0];

%Time vector
t=[0 5];
%the minimum difference in seconds between times to compute q(t) functions
delta_t=0.05;

% B)COMPLETE THE FUNCTION second_order DEFINED BELOW
figure, xlabel('t (s)'), ylabel('q (rad), qd (rad/s), qdd (rad/s/s)'), title('FIFTH ORDER PLANNER'), hold on
[q_t, qd_t, qdd_t, time, k]=fifth_order([q(1) q(2)], [qd(1) qd(2)], [qdd(1) qdd(2)], [t(1) t(2)], delta_t);
plot(time, q_t, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
legend('Position (rad)','Speed (rad/s)', 'Acceleration (rad/s/s)')

%WRITE EQUATION
disp('The equation computed can be written as:')
fprintf('\n q(t)= k1 + k2*t + k3*t^2 + k4*t^3 + k5*t^4 + k6*t^5')
fprintf('\n with k=')
k


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 5:
%   Use the results from exercise 4 to animate a 6 DOF robot with the joint 
%   trajectories of the 4th order planner computed before.
%
%   This exercise is solved as an example for students.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exercise5(qt)
%load a 6DOF robot from the library, such as the IRB6640
robot = load_robot
%adjust 3D view as desired
adjust_view(robot)

%call exercise4. 0.01 is a time param for the simulation. Adjust it to your
%needs
qt=exercise4(0.01);
%repeat the joint computed 6 times for all the joints!
Qt = [qt; qt; qt; qt; qt; qt];
%animate the trajectory on the robot
animate(robot, Qt);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Exercise 4:
%   a) Use a 4th order planner to compute a trapezoid acceleration profile.
%       The inputs for this problem are:
%           qini, qfinal: the initial and final joint position (rad)
%           amax: max. angular acceleration allowed
%           wmax: max. angular speed allowed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function qt = exercise6(delta_t)
% Go from q1 to q2 rad q=[q1 q2 ];
qini=0.1; %rad
qfinal=pi/4; %rad (TRY ALSO WITH 0.8)
amax = 5; %rad/s/s
wmax = 1; %rad/s

% compute total displacement
qT = qfinal-qini;
%compute time to accelerate
t1 = abs(wmax/amax);
%compute time at constant speed
tcte = (abs(qT)-abs(amax)*t1^2)/wmax;

%this is the case of a triangle profile, in which the sector at constant
%speed does not exist
if (tcte <= 0)
    disp('triangle profile')
    tT = sqrt(4*qT/amax);
    %Now construct the time vector
    t = [0 tT/2 tT/2 tT];
    %Now construct the q vector
    q1 = qini;
    q2 = qini + qT/2;
    q3 = q2;
    q4 = qfinal; 
    q = [q1 q2 q3 q4];
    %recompute the new max speed
    wmax = amax*tT/2;
    figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s), q_d (rad/s/s)'), title('FOURTH ORDER PLANNER. EXERCISE 4. TRAP. PROFILE'), hold on
    [q_t1, qd_t, qdd_t, time, k]=fourth_order([q(1) q(2)], [0 wmax],   amax, [t(1) t(2)], delta_t); plot(time, q_t1, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
    [q_t2, qd_t, qdd_t, time, k]=fourth_order([q(3) q(4)], [wmax 0],  -amax, [t(3) t(4)], delta_t); plot(time, q_t2, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
    legend('Joint position q (rad)', 'Joint speed qd (rad/s)', 'Joint accel qdd (rad/s/s)')
    qt = [q_t1 q_t2];
else
    %Now construct the time vector
    t = [0 t1 t1+tcte 2*t1+tcte];
    %Now construct the q vector
    q1 = qini;
    q2 = qini + 0.5*amax*t1^2;
    q3 = qini + 0.5*amax*t1^2 + wmax*tcte;
    q4 = qfinal; 
    q = [q1 q2 q3 q4];

    figure, xlabel('t (s)'), ylabel('q (rad), q_d (rad/s), q_d (rad/s/s)'), title('FOURTH ORDER PLANNER. EXERCISE 4. TRAP. PROFILE'), hold on
    [q_t1, qd_t, qdd_t, time, k]=fourth_order([q(1) q(2)], [0 wmax],   amax, [t(1) t(2)], delta_t); plot(time, q_t1, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
    [q_t2, qd_t, qdd_t, time, k]=fourth_order([q(2) q(3)], [wmax wmax], 0,   [t(2) t(3)], delta_t); plot(time, q_t2, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
    [q_t3, qd_t, qdd_t, time, k]=fourth_order([q(3) q(4)], [wmax 0],  -amax, [t(3) t(4)], delta_t); plot(time, q_t3, 'r'), plot(time, qd_t, 'g'), plot(time, qdd_t, 'b')
    legend('Joint position q (rad)', 'Joint speed qd (rad/s)', 'Joint accel qdd (rad/s/s)')
    qt = [q_t1 q_t2 q_t3];
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First order interpolator.
%   Given the start and end joint positions and times compute a first order
%   polinomial of the form:
%   q(t) = k(1) + k(2)*t
%
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       t: a time vector with t = [t1 t2]
%       delta_t: the sample time between computed q(t)'s
%   Returns:
%      
%       q: the values of q(t) as a function of the time vector used.
%       qd: the speed qd(t) as a function of the time vector (which is a constant in this case)
%       t: the time vector used.
%       k: the polynomial coefficients k that allow to compute q(t) as:
%               q(t) = k(1) + k(2)*t
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_t, qd_t, time, k]=first_order(q, t, delta_t)
%define the time matrix
A = [1 t(1); 
     1 t(2)];
b = [q(1) q(2)];
% compute the coefficients of k for a first order planner
k=inv(A)*b';

%define the time vector using delta_time
time=t(1):delta_t:t(2);
%the first order equation
q_t=k(1) + k(2)*time;
%return a constant speed
qd_t=k(2)*ones(1,length(time));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Second order planner:
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       qd: the initial speed at t1.
%       t: a time vector with t = [t1 t2]
%       delta_t: the sample time between computed q(t)'s
%
%   Returns:
%       q: the values of q(t) as a function of the time vector used.
%       qd: the speed qd(t) as a function of the time vector (which is a constant in this case)
%       t: the time vector used.
%       k: the polynomial coefficients k that allow to compute q(t) as:
%               q(t) = k(1) + k(2)*t + k(3)*t^2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_t, qd_t, qdd_t, time, k]=second_order(q, qd, t, delta_t)

k=inv([1 t(1) t(1)^2; 1 t(2) t(2)^2; 0 1 2*t(1)])*[q(1) q(2) qd(1)]';

time=t(1):delta_t:t(2);
q_t=k(1)+k(2)*time+k(3)*time.^2;
qd_t=k(2)*ones(1,length(time)) + 2*k(3)*time;
qdd_t=2*k(3)*ones(1,length(time));



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Third order interpolator.
%   Given the:
%       - start and end joint positions.
%       - start speed.
%       - start acceleration.
%       - and start and end times,
%       
%   Compute a third order polynomial of the form:
%   q(t) = k(1) + k(2)*t + k(3)*t^2 + k(4)*t^3
%   
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       qd: start speed
%       qdd: start acceleration.
%       t: a time vector with t_i = [t1 t2]  
%       
%
%   Returns:
%       
%       q_t: the values of q(t) as a function of the time vector used.
%       the speed qd(t) as a function of the time vector (which is a constant in this case)
%       time: the time vector used.
%       ki: the polynomial coefficients k that allow to compute q(t) as:
%       q(t) = k(1) + k(2)*t
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_t, qd_t, qdd_t, time, k]=third_order(q, qd, qdd, t, delta_t)
%   third order
A=[1 t(1) t(1)^2 t(1)^3;
   1 t(2) t(2)^2 t(2)^3;
   0   1  2*t(1) 3*t(1)^2;
   0   0   2     6*t(1)];
k=inv(A)*[q(1) q(2) qd(1) qdd(1)]';

time=t(1):delta_t:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3;
qd_t= k(2)*ones(1,length(time)) + 2*k(3)*time + 3*k(4)*time.^2;
qdd_t=2*k(3)*ones(1,length(time)) + 6*k(4)*time;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fourth order interpolator.
%   Given the:
%       - start and end joint positions.
%       - start and end speed.
%       - start acceleration.
%       - and start and end times,
%       
%   Compute a fourth order polynomial of the form:
%   q(t) = k(1) + k(2)*t + k(3)*t^2 + k(4)*t^3 + k(5)*t^4 
%   
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       qd: start speed
%       qdd: start acceleration.
%       t: a time vector with t_i = [t1 t2]  
%       
%
%   Returns:
%       q_t: the values of q(t) as a function of the time vector used.
%       the speed qd(t) as a function of the time vector (which is a constant in this case)
%       time: the time vector used.
%       ki: the polynomial coefficients k that allow to compute q(t) as:
%       q(t) = k(1) + k(2)*t
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_t, qd_t, qdd_t, time, k]=fourth_order(q, qd, qdd, t, delta_t)
%   third order
A=[1 t(1) t(1)^2 t(1)^3 t(1)^4;
   1 t(2) t(2)^2 t(2)^3 t(2)^4;
   0   1  2*t(1) 3*t(1)^2 4*t(1)^3;
   0   1  2*t(2) 3*t(2)^2 4*t(2)^3;
   0   0   2     6*t(2)   12*t(2)^2];
k=inv(A)*[q(1) q(2) qd(1) qd(2) qdd(1)]';

time=t(1):delta_t:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3 + k(5)*time.^4;
qd_t= k(2)*ones(1,length(time)) + 2*k(3)*time + 3*k(4)*time.^2 + 4*k(5)*time.^3;
qdd_t=2*k(3)*ones(1,length(time)) + 6*k(4)*time + 12*k(5)*time.^2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fifth order interpolator.
%   Given the:
%       - start and end joint positions.
%       - start and end speed.
%       - start and end acceleration.
%       - and start and end times,
%       
%   Computes a fifth order polynomial of the form:
%   q(t) = k(1) + k(2)*t + k(3)*t^2 + k(4)*t^3 + k(5)*t^4 + k(6)*t^5
%   
%   Inputs:
%       q: a joint vector of two inputs q = [q1 q2]
%       qd: start and end speed qd = [qd1 qd2]
%       qdd: start and end acceleration. qdd = [qdd1 qdd2]
%       t: a time vector with t_i = [t1 t2]  
%       
%
%   Returns:
%       q_t: the values of q(t) as a function of the time vector used.
%       the speed qd(t) as a function of the time vector 
%       the acceleration qdd(t) as a functiono f time.
%       time: the time vector used.
%       ki: the polynomial coefficients k that allow to compute q(t) as:
%       q(t) = k(1) + k(2)*t + k(3)*t^2 + k(4)*t^3 + k(5)*t^4 + k(6)*t^5
%   
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_t, qd_t, qdd_t, time, k]=fifth_order(q, qd, qdd, t, delta_t)
%   quinto orden
A=[1 t(1) t(1)^2 t(1)^3 t(1)^4 t(1)^5;
   1 t(2) t(2)^2 t(2)^3 t(2)^4 t(2)^5;
   0   1  2*t(1) 3*t(1)^2 4*t(1)^3 5*t(1)^4;
   0   1  2*t(2) 3*t(2)^2 4*t(2)^3 5*t(2)^4;
   0   0   2     6*t(1)   12*t(1)^2 20*t(1)^3;
   0   0   2     6*t(2)   12*t(2)^2 20*t(2)^3];
% la ecuacion es:
% A*k = b
% donde b es un vector de las condiciones de contorno
% posicion inicial y final
% velocidad inicial y final
% aceleraciones inicial y final
b(1) = q(1);
b(2) = q(2);
b(3) = qd(1);
b(4) = qd(2);
b(5) = qdd(1);
b(6) = qdd(2);

%k = [k(1) k(2) k(3) k(4) k(5) k(6)];
k = inv(A)*b(:);

% ecuaci�n: q(t) = k1+k2*t+k3*t^2+k4*t^3+k5*t^4+k6*t^5
%           qd(t) = k2 + 2*k3*t + 3*k4*t^2 + 4*k5*t^3 + 5*k6*t^4
%           qdd(t) = 2*k3 + 6*k4*t + 12*k5*t^2 + 20*k6*t^3
time = t(1):delta_t:t(2);
q_t = k(1) + k(2)*time + k(3)*time.^2 + k(4)*time.^3 + k(5)*time.^4 + k(6)*time.^5;
qd_t= k(2) + 2*k(3)*time + 3*k(4)*time.^2 + 4*k(5)*time.^3 + 5*k(6)*time.^4;
qdd_t=2*k(3) + 6*k(4)*time + 12*k(5)*time.^2 + 20*k(6)*time.^3;




