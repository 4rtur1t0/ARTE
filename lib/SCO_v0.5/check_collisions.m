%check collisions
function collision = check_collisions(pathq)
collision = 0; %no collision
global parameters
for j=1:size(pathq, 2)
    [Po] = cost_function_distance_4dof(pathq(:,j));
    %Po
    if Po < 0.5%parameters.check_collision_weight
        collision = 1; %yes! collision detected
        break
    end
end