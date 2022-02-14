
function test_T2quaternion()
close all

T1 = [-1 0 0 0;
       0 1 0 0;
       0 0 -1 0;
       0 0  0 1]
  
T2 = [-0.5      0   0.845   0;
        0       1   0       0;
      -0.845    0  -0.5    0;
       0        0   0 1]
figure, hold
draw_axes(T1, 'X1', 'Y1', 'Z1', 1)
draw_axes(T2, 'X2', 'Y2', 'Z2', 1)   
axis equal

Q1a = T2quaternion(T1)
Q1b = T2quaternion2(T1)
Q1c = T2quaternion_shepherd(T1)

Q2a = T2quaternion(T2)
Q2b = T2quaternion2(T2)
Q2c = T2quaternion_shepherd(T2)

Q1a - Q1b
Q2a - Q2b

%w = angular_w_between_quaternions(Q1, Q2, 1)



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


  