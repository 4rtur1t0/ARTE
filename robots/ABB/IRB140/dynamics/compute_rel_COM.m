% COMPUTE THE relative transformation between
% the DH system i and COM of link i.
function compute_rel_COM
robot = load_robot('ABB', 'IRB140');
% at the initial position of the robot, 
% annotate the global position of each COM link with
% respect to the global reference frame
abs_rcom = [0.01768, 0.01311, 0.27238; % link 1
            0.06127, -0.03876,0.54161; % link 2
            0.12202, 0.00105, 0.70911; % link 3
            0.4135, -0.00046, 0.71211; % link 4
            0.45, 0.0, 0.71211;% link 5
            0.5105, 0.0000, 0.71211]; % link 6
% at the initial position of the robot, annotate
% the global orientation of each COM link with
% respect to the global reference frame
abs_eul = [0, 0, 0; % link1
           0, 0, 0; % link2
           0, 0, 0; % link3
           0, 0, 0; % link4
           0, -pi/2, 0; % link 5
           0, -pi/2, 0];  % link 6
% the relative vectors and orientations (in Euler angles xyz)
 r_com = [];
 euler_com1 = [];
 euler_com2 = [];
 A = eye(4);
 q = [0 0 0 0 0 0];
 for i=1:robot.DOF
     % compute A0i
     A = A*dh(robot, q, i);
     % compute A0_comi
     A0_comi = eye(4);
     A0_comi(1:3,4)=abs_rcom(i,:)';
     R = euler2rot(abs_eul(i, :), 'XYZ');
     A0_comi(1:3,1:3) = R;
     % compute the rel transform
     Ai_comi = inv(A)*A0_comi;
     % Store results
     r_com = [r_com; Ai_comi(1:3, 4)'];
     [e1, e2] = rot2euler(Ai_comi, 'XYZ');
     euler_com1 = [euler_com1; e1];
     euler_com2 = [euler_com2; e2];
 end
 r_com
 euler_com1
 euler_com2
 