% compute a trapezoidal profile
function [q_t, qd_t, qdd_t, time] = compute_trapezoidal_profile(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    [total_time]=find_total_time(q_initial, qd_initial, q_final, qd_final, qdmax, taccel);
    %now find the speed for each joint
    [qd_max_movement] = compute_speeds(q_initial, qd_initial, q_final, qd_final, taccel, total_time);
    % find max time and replan with that
    [q_t, qd_t, qdd_t, time]=trapezoidal(q_initial, qd_initial, q_final, qd_final, qd_max_movement, taccel, total_time);
end



% Compute minimum time to complete movement
function [total_time_movement]=find_total_time(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    global configuration
    %caution: qdmax expresses the magnitude of the max speed for
    %each joint
    %compute movement direction and q
    movement = q_final-q_initial;
    qdmax = sign(movement).*abs(qdmax);
    %compute initial and final acceleration
    %caution: taccel must be a a product of delta_time. E.g. 3xdelta_time
    qddini = (qdmax - qd_initial)/taccel;
    qddfinal = (qd_final - qdmax)/taccel;

    dqI = qd_initial.*taccel + (qddini/2).*taccel^2;
    dqI = abs(dqI);
    dqIII = qdmax.*taccel + (qddfinal/2).*taccel^2;
    dqIII = abs(dqIII);
    dq_total = abs(q_final-q_initial);
    dqII = dq_total - dqI - dqIII;

    % T should be positive for a trapezoidal profile if T is negative for any joint, then a replanning must be done to
    % complete a triangular profile
    % caution qdmax must be modified so that 
    %- taccel is mod configuration.delta_time
    %- T is mod configuration.delta_time
    T = (dq_total - dqI - dqIII)./abs(qdmax);
    
    max_time_constant = max(T);
    %this is the standard trapezoidal profile, with a time at constant
    %speed
%    if max_time_constant > configuration.delta_time
    n = ceil(max_time_constant/configuration.delta_time);
    %this is the max time at constant speed mod delta_time
    max_time_constant = n*configuration.delta_time;
    total_time_movement = max_time_constant + 2*taccel;
end

function [qd_max_movement] = compute_speeds(q_initial, qd_initial, q_final, qd_final, taccel, total_time)
    t_constant = total_time - 2*taccel;
    dq = abs(q_final-q_initial);
    qd_max_movement = (dq - 0.5*(qd_initial+qd_final)*t_constant)/(taccel+t_constant);
    movement = q_final-q_initial;
    qd_max_movement= sign(movement).*abs(qd_max_movement);
end

% given the time, compute the speed of the trapezoidal profile
function [q_t, qd_t, qdd_t, time]=trapezoidal(q_initial, qd_initial, q_final, qd_final, qdmax, taccel, time_movement)
    % 
    time_constant = time_movement - 2*taccel;
    % CAUTION: some of th ejoints will be planned with a trapezoidal
    % profile, some other with a triangular profile
    [q1_t, qd1_t, qdd1_t, time1] = first_segment(q_initial, qd_initial, qdmax, taccel);
    
        % second segment at constant speed
    [q2_t, qd2_t, qdd2_t, time2] = second_segment(q1_t(:,end), qd1_t(:,end), time1(end), time_constant);
    if ~isempty(q2_t)
        %third segment at constant acceleration
        [q3_t, qd3_t, qdd3_t, time3] = third_segment(q2_t(:,end), qd2_t(:,end), q_final, qd_final, time2(end), time_movement, taccel);
    else
        %third segment at constant acceleration
        [q3_t, qd3_t, qdd3_t, time3] = third_segment(q1_t(:,end), qd1_t(:,end), q_final, qd_final, time1(end), time_movement, taccel);
    end
    
    
    q_t = [q1_t q2_t q3_t];
    qd_t = [qd1_t qd2_t qd3_t];
    qdd_t = [qdd1_t qdd2_t qdd3_t];
    time = [time1 time2 time3];  
%    
%     close all
%     figure,
%     plot(time, q_t),
%     figure
%     plot(time, qd_t)
%     close all       
end

function [q_t, qd_t, qdd_t, time] = first_segment(q_initial, qd_initial, qdmax, taccel)
    global configuration
    delta_time = configuration.delta_time;
    time = 0:delta_time:taccel;

    qdd_initial = (qdmax - qd_initial)/taccel;
    
    a0 = q_initial;
    a1 = qd_initial;
    a2 = qdd_initial/2;
    
    q_t = a0 + a1.*time + a2.*time.^2;
    qd_t = a1 + 2*a2.*time;
    qdd_t = 2*a2*ones(1,length(time));
end

% Plan a second segment at constant speed
function [q_t, qd_t, qdd_t, time] = second_segment(q_initial, qd_initial, time_start, time_constant_speed)
    global configuration
    delta_time = configuration.delta_time;
    % caution, the total movement at constant speed is reduced by
    time = delta_time:delta_time:(time_constant_speed);
    
    % offset time
    time = time + time_start;
    
    a0 = q_initial - qd_initial*time_start;
    a1 = qd_initial;

    q_t = a0 + a1.*time;
    qd_t = a1*ones(1, length(time));
    qdd_t = zeros(length(qd_initial), length(time));
end

function [q_t, qd_t, qdd_t, time] = third_segment(q_initial, qd_initial, q_final, qd_final, time_start, time_end, taccel)
    global configuration
    delta_time = configuration.delta_time;
    time = delta_time:delta_time:(taccel);
    
    % offset time so that start and stop times are not repeated
    time = time + time_start ;
    
    qdd_final = (qd_final - qd_initial)/taccel;
    
    time_d = time_end;
    a2 = qdd_final/2;
    a1 =  qd_initial -2*a2*time_start;
    a0 = q_final - a1*time_d - a2*time_d^2;
        
    q_t = a0 + a1.*time + a2.*time.^2;
    qd_t = a1 + 2*a2.*time;
    qdd_t = qdd_final*ones(1,length(time));
end
