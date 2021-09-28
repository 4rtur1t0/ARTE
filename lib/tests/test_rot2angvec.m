
function test_rot2angvec

convention = 'XYZ'
alpha = pi/2;
beta = 0;
gamma = pi/2;

R = Rot(alpha, 'x')*Rot(beta, 'y')*Rot(gamma, 'z')

[V,D] = eig(R)

% % look for the eigenvector that corresponds to the unique real eigenvalue
for i=1:3
    % if imaginary part is zero
     if imag(D(i,i)) == 0
         vrot = V(:, i)
     end
end
vrot = V(:, 3)
% by definition, the vector is unchanged in the rotation
R*vrot

vrot

% compute theta
th = acos((trace(R)-1)/2)


R
V
D

'equals to'

vrot 
th

%Obtain R again
R1 = Rodrigues(th*vrot)
R2 = Rodrigues(-th*vrot)

