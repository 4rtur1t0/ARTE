function txt = moveDataTip_smm(~,event_obj,hFigure)

event_obj

xyz = event_obj.Position;
txt = xyz;

global robot T smm ps qs u

% qs = [];
% ps = [];

smm_copy = smm;
smm_copy = round(smm_copy*10000)/10000;
xyz = round(xyz*10000)/10000;
% q = smm(event_obj.index,:)';
RowIdx = find(ismember(smm_copy(:,[1,2,6]), xyz,'rows'))

q = smm(RowIdx,:)';

Tsmm = directkinematic(robot,q);

subplot(1,2,1);
xlim([-0.9,0.2])
ylim([-0.5,0.5])
zlim([-0.5,0.5])

if isreal(q)

    % drawrobot3d(robot,qinv(:,1),1)

    if exist('ps','var')
        delete(ps)
    end

    if exist('qs','var')
        delete(qs)
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

        % light;

        %change material properties
        material( [0.5 0.5 0.01]);
        daspect([1 1 1])

        if i<= robot.DOF
            Tloc=Tloc*dh(robot, q, i);
        end
    end

    hold on
    qs(1) = quiver3(Tsmm(1,4),Tsmm(2,4),Tsmm(3,4),Tsmm(1,1),Tsmm(2,1),Tsmm(3,1),0.1,'r');
    qs(2) = quiver3(Tsmm(1,4),Tsmm(2,4),Tsmm(3,4),Tsmm(1,2),Tsmm(2,2),Tsmm(3,2),0.1,'g');
    qs(3) = quiver3(Tsmm(1,4),Tsmm(2,4),Tsmm(3,4),Tsmm(1,3),Tsmm(2,3),Tsmm(3,3),0.1,'b');
    v = Tsmm(1:3,1:3)*u;
    qs(4) = quiver3(Tsmm(1,4),Tsmm(2,4),Tsmm(3,4),v(1)/norm(v),v(2)/norm(v),v(3)/norm(v),0.2,'k');
    hold off

else

    disp('The current solution cannot reach this pose.')

end