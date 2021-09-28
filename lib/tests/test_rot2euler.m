
function test_rot2euler
testXYZ()
%testZYX()

function testXYZ()
convention = 'XYZ'
alpha = pi/2;
beta = pi/2;
gamma = pi/2;
euler_initial = [alpha beta gamma]

R = Rot(alpha, 'x')*Rot(beta, 'y')*Rot(gamma, 'z')

[sol1, sol2] = rot2euler(R,convention)

%check both solutions yield the same R
R1 = Rot(sol1(1), 'x')*Rot(sol1(2), 'y')*Rot(sol1(3), 'z')
R2 = Rot(sol2(1), 'x')*Rot(sol2(2), 'y')*Rot(sol2(3), 'z')

error1 = norm(R-R1)
error2 = norm(R-R2)


function testZYX()
% test ZYX convention
convention = 'ZYX'
alpha = 0;
beta = pi/2;
gamma = -pi/4;
euler_initial = [alpha beta gamma]

R = Rot(alpha, 'z')*Rot(beta, 'y')*Rot(gamma, 'x')

[sol1, sol2] = rot2euler(R,convention);

%check both solutions yield the same R
R1 = Rot(sol1(1), 'z')*Rot(sol1(2), 'y')*Rot(sol1(3), 'x');
R2 = Rot(sol2(1), 'z')*Rot(sol2(2), 'y')*Rot(sol2(3), 'x');

error1 = norm(R-R1)
error2 = norm(R-R2)



