%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ComputeLPath
%   Moves robot to a particular joint coordinates%
% CAUTION: NOT TO BE CONFUSED WITH THE RAPID FUNCTIONS
% Once
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
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

function [qt, qdt] = ComputeLPath(robot, Tf, speed_percent)
    global configuration
    delta_time = configuration.delta_time;
    lin_vel = speed_percent*robot.linear_velmax/100.0;    
    lin_vel = 200*robot.linear_velmax/100.0;
    
    
    
    %start joint coordinates, as saved before
    q0 = robot.q;
    
    T0 = directkinematic(robot, q0);
    p0 = T0(1:3, 4);
    pf = Tf(1:3, 4);
    
    %caution: a trapezoidal profile should be used
    dist = norm(pf-p0);
    time = dist/lin_vel;
    
    qinv = inversekinematic(robot, Tf);
    %qf = qinv(:,1);
    %Q0 = T2quaternion(T0);
    Qf = T2quaternion(Tf);
    
    
    % Perform a path planning speed
    [trap_pos, trap_vel, trap_accel, time]= compute_trapezoidal_profile(0, 0, dist, 0, lin_vel, 0.1);
    
    trap_vel = 1;
    qt = [robot.q];
    qdt = [robot.qd];
    q = q0;
    rr = [];
    v = [];
    qn = [];
    close all
    i = 2;
    while 1
        Ti = directkinematic(robot, q);
        Qi = T2quaternion(Ti);
        pi = Ti(1:3,4);
        vi = compute_speed(pi, pf, 1);
        wi = compute_ang_speed(Qi, Qf, 1);
        lin_vel_i = trap_vel; %(i);

        Vref = [vi' wi']';
        J = manipulator_jacobian(robot, q);
        
        Vref = normalize_linear_speed(vi, wi, lin_vel_i);
        
        %rr = [rr rank(J)];
        % check rank of J
        % check det(J)
        v = [v norm(Vref)];
        % a trapezoidal speed profile should be used
        
        
        qd = inv(J)*Vref;
        qn = [qn max(abs(qd))];
        % check max speed of each joint
        
        qd = normalize_to_max_joint_speed(robot, qd);
        
        q = q + qd*delta_time;

        % find the error of p with respect to the line.
        [delta_end, error_line, error_line_vector] = find_errors(p0, pf, pi); 
        delta_end
        error_line

        if delta_end < 0.05
            break
        end
         
        qt = [qt q];
        qdt = [qdt qd];       
        %time = time-delta_time;
        i = i + 1;
    end

%figure, plot(rr)
figure, plot(v)
figure, plot(qn)
end


function vref = normalize_linear_speed(vi, wi, lin_vel_i)
  if norm(vi) > 0
       vi = lin_vel_i*vi/norm(vi);
  end
  if norm(wi) > 0
       wi = lin_vel_i*wi/norm(wi);
  end
  vref = [vi(:); wi(:)];

end

function qd = normalize_to_max_joint_speed(robot, qd)

    %[index, val] = max(abs(qd));
    %diff = abs(robot.maxvel)-abs(qd);
    ratio = abs(qd)./abs(robot.velmax)
    [value, index] = max(ratio);
    %saturate max value to 1 in ratio
    if value > 1
        qd = qd/value;
    end
end
   
% find_errors:
% error_end: the error with respect to the end point
% error_line: the error of p to the line defined b point a and vector n.
% the line is defined by points a and b.
function [error_end, error_line, error_line_vector]=find_errors(a, b, p)
error_end = sqrt((p-b)'*(p-b));

% define the line as a, n
n = (b-a);
n = n/norm(n);

error_line = (a-p)-((a-p)'*n)*n;
error_line = norm(error_line);

% Now obtain current point minus initial point a
% project
w = p-a;
p_ = n*dot(w, n);
% add the origin since the point p_ is referred to the line
p_ = p_ + a;
% find a vector connecting the current point p and the point belonging to
% the line p_
error_line_vector = p_ - p;
end


% % find a delta_q that causes the error to the line to become zero
% function correct_error(p0, pf, pi, q)
% % find the error of p with respect to the line.
% [delta_end, error_line, error_line_vector] = find_errors(p0, pf, pi);
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_speed(Pi, Pf, time)
    %compute a constant linear speed till target
    v = (Pf-Pi)/time;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posici�n y orientaci�n final, calcular cu�les deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [w] = compute_ang_speed(Qi, Qf, time)
%compute a constant angular speed till target
%asume the movement is performed in 1 second
w = angular_w_between_quaternions(Qi, Qf, time);
w = w(:);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute angular speed w that moves Q0 into Q1 in time total_time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = angular_w_between_quaternions(Q0, Q1, total_time)
%below this number, the axis is considered as [1 0 0]
%this is to avoid numerical errors
%this is the actual error allowed for w
%epsilon_len = robot.parameters.epsilonQ;
epsilon_len = 0.0001;
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);

if len > epsilon_len
    angle=2*atan2(len, Q(1));
    axis=[Q(2) Q(3) Q(4)]./len;
else
    angle=0;
    axis=[1 0 0];
end
w=axis*angle/total_time;
end