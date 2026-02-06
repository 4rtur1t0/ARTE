robot = load_robot('example', 'example_RRR');
q = [pi/4 pi/4 pi/4];
T = directkinematic(robot, q)

qi = inversekinematic(robot, T)

for i=1:4    
   qt = qi(:,i);
   Ti = directkinematic(robot, qt);
   Tt=T-Ti
end