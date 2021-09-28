% The example shows how to transforms between different conventions 
%  of Euler angles. In particular from ZYZ to ZYX
%
%
%
%
function test_transform_euler_angles()
% initial
convention1 = 'ZYZ'
alpha1 = pi/4;
beta1 = pi/4;
gamma1 = pi/4;
euler1 = [alpha1 beta1 gamma1]

R1 = Rot(alpha1, 'z')*Rot(beta1, 'y')*Rot(gamma1, 'z')

convention2 = 'ZYX'
[euler2_1, euler2_2] = rot2euler(R1,convention2)

% check that yields the same rotation matrix
R2_1 = Rot(euler2_1(1), 'z')*Rot(euler2_1(2), 'y')*Rot(euler2_1(3), 'x')
R2_2 = Rot(euler2_2(1), 'z')*Rot(euler2_2(2), 'y')*Rot(euler2_2(3), 'x')


norm(R1-R2_1)
norm(R1-R2_2)