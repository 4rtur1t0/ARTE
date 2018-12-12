
function manips = compute_manip_4dof(robot, path)

manips = [];
for i=1:size(path,2)
    q = path(:,i);
    J = manipulator_jacobian(robot, q);
    % moving on vx, vy and wz!!!
    J = [J(1:2,:); J(6,:)];
    manip = sqrt(det(J*J'));
    manips = [manips manip];
end



