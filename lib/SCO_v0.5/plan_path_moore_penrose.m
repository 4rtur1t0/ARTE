%
%    PLAN A BASIC PATH USING MOORE PENROSE PSEUDO INVERSE
%
function [pathq, pathT] = plan_path_moore_penrose(robot, q0)
% %initial pose and manipulability
global parameters
pathq = [];
pathT = build_initial_path();
q = q0;
%normalize initial
q = atan2(sin(q), cos(q));
for i=1:size(pathT,2)
    fprintf('Moore-penrose: Move %d out of %d\n', i, size(pathT,2))
    %update to next point in trajectory
    %obtain it from pathT cell array
    Ti = pathT{i}.T;
%     if robot.DOF == 4
%         q = inversekinematic_4dofplanar(robot, Ti, q);
%     else
%         q = inverse_kinematics_sawyer(robot, Ti, q);
%     end
    q = inversekinematic(robot, Ti, q);
    pathq = [pathq q];
    if parameters.animate
        drawrobot3d(robot, q)
        draw_axes(Ti, 'Xpiece', 'Ypiece', 'Zpiece', 1.2);
    end
end

if parameters.animate
    animate_local(robot, pathq)
    manips = compute_manip(robot, pathq);
    figure,
    plot(manips)
    title('manipulability index at each movement: initial path')
    figure,
    plot(pathq')
    title('joint positions')
    legend('q_1', 'q_2','q_3','q_4','q_5','q_6','q_7')
end

