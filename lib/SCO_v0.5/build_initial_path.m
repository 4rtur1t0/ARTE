
function pathG = build_initial_path()
global parameters
pathG=[];
j=1;
for i=1:size(parameters.trajectory,2)
    line = parameters.trajectory{i}.line;
    p0 = line(:,1);
    p1 = line(:,2);
    T = parameters.trajectory{i}.T0;
    
    partial_path = build_partial_path(p0, p1, T);
    %append
    for k=1:size(partial_path,2)
        pathG{j}.T=partial_path{k}.T;
        j=j+1;
    end
end




function partial_path = build_partial_path(p0, pf, T0)
global parameters
%this defines the line to follow in direction
deltaV = pf - p0;
deltaV = deltaV/norm(deltaV);
%and total length of the movement
total_length = norm(pf - p0); %m
%linspace!!
mov = linspace(0, total_length, parameters.N);
%line0 = [p0 pf];
%plot(line0(1,:),line0(2,:),'k')

partial_path=[];
Ti = T0;
for i=1:length(mov)    
    fprintf('Fast global plan %d out of %d\n', i, length(mov))
    %update to next point in trajectory
    Ti(1:3,4) = p0 + mov(i)*deltaV;   
    partial_path{i}.T = Ti;
end