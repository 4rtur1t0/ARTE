function draw_me(robot, q)
close all
figure, hold
origin = [0 0 0];
T = robot.T0;
color = ['r' 'g' 'b' 'c' 'm' 'y' 'k'];
axis equal
for i=1:robot.DOF+1      
    destination=T(1:3,4);
    col = randperm(6);
    %line([origin(1) destination(1)],[origin(2) destination(2)],[origin(3) destination(3)], 'Color',  color(col(1)) ,'LineWidth', 3 );
    origin = destination;
    
    draw_axes(T, sprintf('X_%d',i-1), sprintf('Y_%d',i-1), sprintf('Z_%d',i-1), robot.graphical.axes_scale);
    if i <= robot.DOF
        T=T*dh(robot, q, i);
    end
end