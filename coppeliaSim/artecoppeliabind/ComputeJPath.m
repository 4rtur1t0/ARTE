% Synchronous path in the joint space, given a T.
% the initial configuration is given as the one that is closest to the
% Q_config
function[q_t, qd_t, qdd_t, time] = JPath(robot, T, target_joint_speed, speed_percent, Q_base_config)
    fprintf('\nCall to JPath ARTE-COPPELIA');

    %obtain current joint coordinates
    q_initial=robot.q;
    qd_initial = robot.qd;
    
    qinv = inversekinematic(robot, T);
    
    dist = qinv - Q_base_config(:);
    dd = [];
    for i=1:size(dist,2)
        d = norm(dist(:,i));
        dd = [dd d];
    end
    [val, index] = min(dd);
    % find minimum dist with Q_base_config
    q1 = qinv(:,index);
    
    %obtain target joint coordinates
    q_final= q1(:); %joint_coord(1:robot.DOF)';
    qd_final = target_joint_speed(:);

    % compute max trapezoidal joint speed
    qdmax = speed_percent*robot.velmax/100;
    taccel = 0.1; %*ones(robot.DOF, 1); % seconds, parameter

    %test joint limits for the final values
    test_joints(robot, q_final);

    % Perform a path planning on joints considering a trapezoidal speed profile
    [q_t, qd_t, qdd_t, time]= compute_trapezoidal_profile(q_initial, qd_initial, q_final, qd_final, qdmax, taccel);
end