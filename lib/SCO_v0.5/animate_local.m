
function animate_local(robot, qq)
global parameters

for i=1:1:size(qq,2)
     drawrobot3d(robot, qq(:,i))    
     for j=1:size(parameters.trajectory,2)
         line_work = parameters.trajectory{j}.line;
         plot3(line_work(1,:),line_work(2,:),line_work(3,:))
     end
     %plot obstacles
     for j=1:size(parameters.obstacles,2)
         points = parameters.obstacles{j}.points;
         v = [points(:,1)'; 
              points(:,2)';
              points(:,3)'];
         f = [1 2 3];
         patch('Faces',f,'Vertices',v,'FaceColor','blue')
     end
     pause(0.05)
end


