function txt = moveDataTip(~,event_obj,hFigure)

clc
xyz = event_obj.Position;
txt = xyz;

global T robot fig ps qinv fig2 solution_number

%fig

T(1,4) = xyz(1);
T(3,4) = xyz(3);
qinv = inversekinematic(robot,T);
q = qinv(:,solution_number);
Q2 = qinv(2,:);

is_real = 1;
for i=1:6
    is_real = isreal(q(i)) && is_real;
    if ~is_real
        break
    end
end

if is_real

    J = compute_jacobian(robot,q);
    condJ = cond(J)
        
    % drawrobot3d(robot,qinv(:,1),1)

    if exist('ps','var')
        delete(ps)
    end

    Tloc = robot.T0;
    for i=1:robot.DOF+1

        V=robot.graphical.link{i}.v;
        V(:,4) = ones(length(V),1); %homogeneous coordinates

        %transform points according to current coordinates
        V = (Tloc*V')';
        V  = V(:,1:3);
        %set robot.graphical.color to add a desired color to your robot
        ps(i) = patch('faces', robot.graphical.link{i}.f, 'vertices', V);
        set(ps(i), 'FaceColor', [0.5 0.6 0.7]);
        set(ps(i), 'EdgeColor','none');

        if i<= robot.DOF
            Tloc=Tloc*dh(robot, q, i);
        end
    end

else

    disp('The current inverse kinematic solution cannot reach this pose.')

end

% fig2
% 
% plot(Q2,'ok')