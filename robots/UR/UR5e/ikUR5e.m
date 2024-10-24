%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Adrian Peidro. Universidad Miguel Hernandez de Elche. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Q = ikUR5e(T)

r11 = T(1,1); r12 = T(1,2); r13 = T(1,3); x = T(1,4);
r21 = T(2,1); r22 = T(2,2); r23 = T(2,3); y = T(2,4);
r31 = T(3,1); r32 = T(3,2); r33 = T(3,3); z = T(3,4);

d1 = 0.1625;
a2 = 0.4250;
a3 = 0.3922;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;

Q = []; % Solutions

% First solve q1 from:
% cos(q1)*(y-d6*r23) + sin(q1)*(d6*r13-x) + (d4) = 0
% This equation has the following form:
% C*cos(q1) + S*sin(q1) + I = 0
% where:
C = y-d6*r23;
S = d6*r13-x;
I = d4;
% It has two solutions for q1:
Q1 = solve_CSI(C,S,I);
for q1=Q1
    % Now solve q5, it has two solutions for each q1:
    q5 = acos(r13*sin(q1) - r23*cos(q1));
    Q5 = [q5,-q5];
    for q5=Q5
        % Now compute the unique solution for q234 that corresponds to a given pair (q1,q5):
        sin_q234 = -r33/sin(q5);
        cos_q234 = -(r13*cos(q1)+r23*sin(q1))/sin(q5);
        q234 = atan2cplx(sin_q234,cos_q234);
        % Now compute the unique solution for q6 that corresponds to a given (q1,q5,q234):
        sin_q6 = (r31*cos(q234) - r32*sin(q234)*cos(q5))/(1 - sin(q234)^2*sin(q5)^2);
        cos_q6 = (r32*cos(q234) + r31*sin(q234)*cos(q5))/(1 - sin(q234)^2*sin(q5)^2);
        q6 = atan2cplx(sin_q6,cos_q6);
        % Now compute the two solutions of q4. First, we compute the following Left-Hand Side:
        LHS = [cos(q234),-sin(q234);sin(q234),cos(q234)]*[-(x*cos(q1)+y*sin(q1));z-d1];
        % q4 has to be solved from:
        % LHS(1) - d6*sin(q5) = k1 = a3*cos(q4) + a2*cos(q3+q4)
        % LHS(2) + d5         = k2 = a3*sin(q4) + a2*sin(q3+q4)
        k1 = LHS(1) - d6*sin(q5);
        k2 = LHS(2) + d5;
        % First we eliminate the cosine and sine of the sum (q3+q4):
        % k1 - a3*cos(q4) = a2*cos(q3+q4)
        % k2 - a3*sin(q4) = a2*sin(q3+q4)
        % Sum the squares of the previous two equations to obtain:
        % (k1 - a3*cos(q4))^2 + (k2 - a3*sin(q4))^2 = a2^2
        % Expand the previous equation to obtain:
        % k1^2 + a3^2*cos(q4)^2 - 2*k1*a3*cos(q4) + k2^2 + a3^2*sin(q4)^2 - 2*k2*a3*sin(q4) - a2^2 = 0
        % (-2*k1*a3)*cos(q4) + (-2*k2*a3)*sin(q4) + (k1^2 + k2^2 + a3^2 - a2^2) = 0
        Q4 = solve_CSI(-2*k1*a3,-2*k2*a3,k1^2 + k2^2 + a3^2 - a2^2);
        for q4=Q4
            % Next, solve the unique solution of q3:
            sin_q34 = (k2 - a3*sin(q4))/a2;
            cos_q34 = (k1 - a3*cos(q4))/a2;
            q34 = atan2cplx(sin_q34,cos_q34);
            q3 = q34 - q4;
            q3 = atan2cplx(sin(q3),cos(q3)); % Wrap to [-pi,pi]. Alternatively, use: q3 = wrapToPi(q3)
            % Finally, solve the unique solution of q2:
            q2 = q234 - q3 - q4;
            q2 = atan2cplx(sin(q2),cos(q2)); % Wrap to [-pi,pi]. Alternatively, use: q2 = wrapToPi(q2)
            Q = [Q,[q1;q2;q3;q4;q5;q6]]; % Add a new solution as a column.
        end
    end
end

end

function x = solve_CSI(C,S,I)
% This function solves x from the equation:
% C*cos(x) + S*sin(x) + I = 0
% This equation can be solved using the following change of variable:
% cos(x) = (1-t^2)/(1+t^2)
% sin(x) = (2*t)/(1+t^2)
% where t = tan(x/2)
t1 = ( sqrt(C^2 - I^2 + S^2) + S)/(C - I);
t2 = (-sqrt(C^2 - I^2 + S^2) + S)/(C - I);
x = 2*atan([t1,t2]);
end

function x = atan2cplx(s,c)
% This function computes atan2 when sine and cosine are complex numbers,
% since the built-in atan2 function of Matlab does not admit complex arguments.
% We want our code to work also when some of the obtained solutions are not
% physically valid, i.e., when some joint angles are complex numbers.
if c>-0.9999 % Threshold for numeric stability.
    x = 2*atan(s/(1+c));
else
    x = pi;
end
end


