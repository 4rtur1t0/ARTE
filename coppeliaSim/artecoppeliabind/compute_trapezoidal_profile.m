% compute a trapezoidal profile
function [q_t, qd_t, qdd_t, time] = compute_trapezoidal_profile(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    [total_time]=find_total_time(q_initial, qd_initial, q_final, qd_final, qdmax, taccel);
    %now find the speed for each joint
    [q_t, qd_t, qdd_t, time]=trapezoidal(q_initial, qd_initial, q_final, qd_final, taccel, total_time); 
end


% Compute minimum time to complete movement
function [total_time_movement]=find_total_time(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    global configuration
    %caution: qdmax expresses the magnitude of the max speed for
    %each joint    
    times_initial = compute_trapezoidal_times(q_initial, qd_initial, q_final, qd_final, qdmax, taccel);
    % The total movement is synchronized to that of the max time for all
    % joints
    max_time_movement = max(times_initial);
    %this is the standard trapezoidal profile, with a time at constant speed
    % round to a multiple of the delta_time
    n = ceil(max_time_movement/configuration.delta_time);
    %this is the max time at constant speed mod delta_time
    max_time_movement = n*configuration.delta_time;  
    total_time_movement = max_time_movement;
end

function times_initial = compute_trapezoidal_times(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    %compute movement direction and q
    movement = q_final-q_initial;
    qdmax = sign(movement).*abs(qdmax);
    times_initial = [];
    for i=1:length(q_initial)
       time = compute_trapezoidal_time(q_initial(i), qd_initial(i), q_final(i), qd_final(i), qdmax(i), taccel);
       times_initial = [times_initial time];
    end

end

function time_total = compute_trapezoidal_time(q_initial, qd_initial, q_final, qd_final, qdmax, taccel)
    global configuration
    
    if qdmax == 0
        time_total = 0;
        return;
    end
    
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
    time_constant = (dq_total - dqI - dqIII)/abs(qdmax);
    if time_constant < configuration.delta_time
        time_constant = 0;
    end
    time_total = time_constant+2*taccel;
end


function [q_t, qd_t, qdd_t, time]=trapezoidal(q_initial, qd_initial, q_final, qd_final, taccel, time_movement)
    global configuration
    time_constant_speed = time_movement - 2*taccel;
    if time_constant_speed > configuration.delta_time
        %normal trapezoidal
        qdmax = compute_trapezoidal_qdmax(q_initial, qd_initial, q_final, qd_final, taccel, time_movement);
        [q1_t, qd1_t, qdd1_t, time1] = first_segment(q_initial, qd_initial, qdmax, taccel);
        [q2_t, qd2_t, qdd2_t, time2] = second_segment(q1_t(:,end), qd1_t(:,end), time1(end), time_constant_speed);
        [q3_t, qd3_t, qdd3_t, time3] = third_segment(q2_t(:,end), qd2_t(:,end), q_final, qd_final, time2(end), time_movement, taccel);
        q_t = [q1_t q2_t q3_t];
        qd_t = [qd1_t qd2_t qd3_t];
        qdd_t = [qdd1_t qdd2_t qdd3_t];
        time = [time1 time2 time3]; 
    else
        %bang-bang trapezoidal (acceleration and deceleration)
        qdmax = compute_bangbang_qdmax(q_initial, qd_initial, q_final, qd_final, taccel);
        [q1_t, qd1_t, qdd1_t, time1] = first_segment(q_initial, qd_initial, qdmax, taccel);
        [q3_t, qd3_t, qdd3_t, time3] = third_segment(q1_t(:,end), qd1_t(:,end), q_final, qd_final, time1(end), time_movement, taccel);
        q_t = [q1_t q3_t];
        qd_t = [qd1_t qd3_t];
        qdd_t = [qdd1_t qdd3_t];
        time = [time1 time3]; 
    end
%     close all
%     figure,
%     plot(time, q_t),
%     figure
%     plot(time, qd_t)
%     close all 
end

% Compute the max speed of the trapezoidal profile considering that there
% exists a time at constant speed.
function qdmax = compute_trapezoidal_qdmax(q_initial, qd_initial, q_final, qd_final, taccel, time_movement)
    dq_total = abs(q_final(:)-q_initial(:));
    tcte = time_movement - 2*taccel;
    qdmax = (dq_total-0.5*(qd_initial(:)+qd_final(:))*tcte)/(taccel+tcte);
    qdmax = sign(q_final(:)-q_initial(:)).*qdmax;
end

%compute the max value of speed qdmax, considering that time at constant
%speed is null
function qdmax = compute_bangbang_qdmax(q_initial, qd_initial, q_final, qd_final, taccel)
    dq_total = abs(q_final(:)-q_initial(:));
    qdmax = (dq_total/taccel)-0.5*(qd_initial(:)+qd_final(:));
    qdmax = sign(q_final(:)-q_initial(:)).*qdmax;
end

function [q_t, qd_t, qdd_t, time] = first_segment(q_initial, qd_initial, qdmax, taccel)
    global configuration
    delta_time = configuration.delta_time;
    time = 0:delta_time:taccel;

    qdd_initial = (qdmax - qd_initial)/taccel;
    
    a0 = q_initial(:);
    a1 = qd_initial(:);
    a2 = qdd_initial(:)/2;
    
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
    
    a0 = q_initial(:) - qd_initial(:)*time_start;
    a1 = qd_initial(:);

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
    
    qdd_final = (qd_final(:) - qd_initial(:))/taccel;
    
    time_d = time_end;
    a2 = qdd_final(:)/2;
    a1 =  qd_initial(:) -2*a2*time_start;
    a0 = q_final(:) - a1*time_d - a2*time_d^2;
        
    q_t = a0 + a1.*time + a2.*time.^2;
    qd_t = a1 + 2*a2.*time;
    qdd_t = qdd_final(:)*ones(1,length(time));
end

