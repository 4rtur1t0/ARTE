
function test_abc_T2quaternion()
close all
%abc=[0 0 0]
%R = Rot(abc(1), 'x')*Rot(abc(2), 'y')*Rot(abc(3), 'z');
R1 = [0 1 0;
     0 0 1;
     1 0 0]
R2 = [0 0 1;
     1 0 0;
     0 1 0]

Q1a = T2quaternion(R1)
Q1b = T2quaternion_shepherd(R1)

Q2a = T2quaternion(R2)
Q2b = T2quaternion_shepherd(R2)

wa = angular_w_between_quaternions(Q1a, Q2a, 1)
wb = angular_w_between_quaternions(Q1b, Q2b, 1)


function w = angular_w_between_quaternions(Q0, Q1, total_time)
% global robot
%below this number, the axis is considered as [1 0 0]
%this is to avoid numerical errors
%this is the actual error allowed for w
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

