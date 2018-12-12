
%
% Computes manipulability of poses given in qs
%
function manips = compute_manip(robot, qs)
manips = [];
for i=1:size(qs,2)
    %compute current manip
    J = manipulator_jacobian(robot, qs(:,i));
    manip = sqrt(det(J*J'));
    manips = [manips manip];
end