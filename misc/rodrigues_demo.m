% Rodrigues' formula (Axis-angle rotation)
% Rotating a vector about an axis in 3D space
% Axis of rotation and the rotation vector have common origin
% Soumitra Sitole
% Reference: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
clc
clear
close all
%% Rotation about an axis passing through the origin:
O = [0,0,0]; % origin
v = [1,1,1]; % v is the vector to be rotated 
K = [4,8,2]; % K denotes the distal point for rotation axis
k = K/norm(K); % k is the normalized rotation axis vector 
figure(1)
plot3([O(1),v(1)],[O(2),v(2)],[O(3),v(3)],'-*r'); % plot initial vector
hold on;
plot3([O(1),K(1)],[O(2),K(2)],[O(3),K(3)],'-k'); % plot rotation axis
for theta = 1:360 % define the angle of rotation (right-handed CS)
    v_rot = v*cosd(theta)+cross(k,v)*sind(theta)+k*(dot(k,v))*(1-cosd(theta)); % rotated vector (Rodrigues' formula)
    plot3([O(1),v_rot(1)],[O(2),v_rot(2)],[O(3),v_rot(3)],':.b');
    hold on;
end
title('Rotation about an axis through the origin');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
%% Rotation about an arbitrary axis:
O = [1,2,0]; % initial point of rotation axis
ov = (v-O); % vector to be rotated
K = [4,8,2]; % K denotes the distal point for rotation axis
ok = (K-O); % ok is the rotation axis vector
k = ok/norm(ok); % k is the normalized rotation axis vector
figure(2)
plot3([O(1),v(1)],[O(2),v(2)],[O(3),v(3)],'-*r'); % plot initial vector
hold on;
plot3([O(1),K(1)],[O(2),K(2)],[O(3),K(3)],'-k'); % plot rotation axis
for theta = 1:360 % define the angle of rotation (right-handed CS)
    v_rot = ov*cosd(theta)+cross(k,ov)*sind(theta)+k*(dot(k,ov))*(1-cosd(theta)); % rotated vector about an axis through global origin(0,0,0)
    v_tran = [O(1)+v_rot(1),O(2)+v_rot(2),O(3)+v_rot(3)]; % transformed vector (translate rotated vector back to initial point) 
    plot3([O(1),v_tran(1)],[O(2),v_tran(2)],[O(3),O(3)+v_rot(3)],':.b');
    hold on;
end
title('Rotation about an arbitrary axis');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal