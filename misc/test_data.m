function test_data()

% pd_0 = [5 5 35]';
% % euler angles of drone
% abc_0 = [-0.1418 0.1404 0.795];
% 
% % Relative vector (Camera-System0)
% tvec_c = [0.01 0 35.89]';
% % Rodrigues vector on Camera reference system
% rvec_c = [-1.1318 2.7325 -0.1117]';

pd_0 = [-5 0 35]';
% euler angles of drone
abc_0 = [0 -0.1404 3.1415926];

% Relative vector (Camera-System0)
tvec_c = [0.01 0 35.70]';
% Rodrigues vector on Camera reference system
rvec_c = [3.1325  0  0.23]';



[error_t, error_angle] = check_sol(pd_0, abc_0, tvec_c, rvec_c)

% 
% function [error_t, error_angle]= check_sol(pd_0, abc_0, tvec_c, rvec_c)
% 
% % Given euler angles of drone (estimate Rotation matrix of 
% % camera based on global Coordinates (System0)
% R0drone = euler2rot(abc_0, 'XYZ');
% R0cam_gt = R0drone*Rot(pi, 'x');
% 
% 
% % Estimate relative rotation matrix Camera-System0 using rvec (Rodrigues)
% Rcam_0 = Rodrigues(rvec_c);
% 
% % Next, compute inverse transform
% % Add a last rotation to match coordinate systems
% R0_cam = Rcam_0'*Rot(pi,'z');
% 
% % The estimation based on the camera observation
% tvec_0 = -R0_cam*tvec_c;
% 
% % 'difference?' --> Error in rotation
% error_R = R0cam_gt-R0_cam
% error_t = pd_0-tvec_0;
% 
% % compute error in angles (gimbal lock, caution)
% % Compute the angles corresponding to each rotation matrix and compute the 
% % difference
% [a1, b1, c1] = rot2euler(R0cam_gt, 'XYZ');
% [a2, b2, c2] = rot2euler(R0_cam, 'XYZ');
% error_angle = [a1, b1, c1] - [a2, b2, c2];
% error_angle = atan2(sin(error_angle), cos(error_angle));



% function Rc = Rodrigues(k)
% th = norm(k);
% k = k/th;
% kx = k(1);
% ky = k(2);
% kz = k(3);
% K = [0  -kz  ky;
%      kz  0  -kx;
%     -ky  kx  0];
% Rc=eye(3) + sin(th)*K + (1-cos(th))*K*K;
