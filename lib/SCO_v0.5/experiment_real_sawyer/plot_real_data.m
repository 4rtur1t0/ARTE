close all
%filename = 'experimento_from_min_vanilla.bag';
filename = 'experimento_from_min_sgo.bag';

%filename = 'experimento_from_max_vanilla.bag';
%filename = 'experimento_from_max_sgo.bag';

%filename = 'moore_penrose.bag';
%filename = 'moore_penrose_max_manip.bag';


bag = rosbag(filename);
bSel = select(bag, 'Topic', 'robot/joint_states');
msgStructs = readMessages(bSel, 'DataFormat', 'struct');
q_joints_struct = cellfun(@(m) double(m.Position),msgStructs, 'UniformOutput', false);
q_joints = [];
for i =1:length(q_joints_struct)
    q_joints = [q_joints q_joints_struct{i}];
end
q_joints = q_joints(2:8, :);
%plot(q_joints')
close all
figure, hold
styles = ['-', '--', ':', '-.', '-', '--', ':', '-.' ]
markers = ['o', '+', '*', 's', 'd', '^', 'p']
colors = ['y', 'm', 'c', 'r', 'g', 'b', 'k']
lln = []

if length(q_joints) >= 7000
    longitude = 7000;
else
    longitude =  length(q_joints);
end

for j=1:7    
    st = strcat(markers(j), colors(j));
    x = 1:300:longitude;
    y = q_joints(j,1:300:longitude);    
    ln = plot(x, y, st, 'LineWidth', 6, 'MarkerSize', 20)         
    lln = [lln ln]
end
xlabel('Trajectory step')
ylabel('Joint positions (rad)')
lgd = legend(lln, 'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7')
%legend(lln)
% now plot all the data
for j=1:7    
    x = 1:1:longitude;
    y = q_joints(j,1:longitude);
    plot(x, y, colors(j), 'LineWidth', 6)         
end

'debug'
% remove unwanted elements from legend
lgd.String=lgd.String(1:7)
