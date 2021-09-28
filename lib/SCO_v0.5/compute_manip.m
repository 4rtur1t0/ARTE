
function manips = compute_manip(robot, path)
%global parameters
manips = [];
for i=1:size(path,2)
    q = path(:,i);
    J = manipulator_jacobian(robot, q);
    % select the proper dofs of J
    J = J(robot.selJ,:);
    manip = sqrt(det(J*J'));
    manips = [manips manip];
end



