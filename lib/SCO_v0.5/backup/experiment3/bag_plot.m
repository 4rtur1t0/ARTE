
filename_bag='experimento_from_min_sgo.bag'
filename_mat='experimento_from_min_sgo.mat'
topic = '/robot/joint_states'
bagMsgs=rosbag(filename_bag);
bagselect = select(bagMsgs,'Topic',topic);

msgStructs = readMessages(bagselect);
pathq=[];
pathqd=[];
for i = 1:size(msgStructs,1)
    msg = msgStructs{i};
    q=msg.Position;
    q_s=q(2:8);
    pathq = [pathq q_s];
    
    qd=msg.Velocity;
    qd_s=qd(2:8);
    pathqd = [pathqd qd_s];
end
figure, plot(pathq')
save(filename_mat)
'ended'
