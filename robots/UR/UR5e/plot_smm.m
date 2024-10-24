close all

global robot T smm ps qs u fig

fig = figure

% drawrobot3d(robot,zeros(1,6));

subplot(1,2,1);
light
grid on

u = [0;1;1]; % direction of the tool in frame of the gripper.
N = 300; % resolution to map self-motion manifolds.

T = directkinematic(robot,zeros(1,6));

solution_number = 3;
step = 0.01;
step_ang = 0.05;

first_plot

while 1
    w = waitforbuttonpress;
    % 28 leftarrow
    % 29 rightarrow
    % 30 uparrow
    % 31 downarrow
    % 43 +
    % 45 -
    % q 113 x+
    % a 97  x-
    % w 119 y+
    % s 115 y-
    % e 101 z+
    % d 100 z-
    % r 114 a+
    % f 102 a-
    % t 116 b+
    % g 103 b-
    % y 121 g+
    % h 104 g-
    % ps = plot3(0,0,0);
    if w
        clc
        value = double(get(gcf,'CurrentCharacter'))
        switch value
            case 49
                solution_number = 1;
            case 50
                solution_number = 2;
            case 51
                solution_number = 3;
            case 52
                solution_number = 4;
            case 53
                solution_number = 5;
            case 54
                solution_number = 6;
            case 55
                solution_number = 7;
            case 56
                solution_number = 8;
            case 28
                T(1,4) = T(1,4) - step;
            case 29
                T(1,4) = T(1,4) + step;
            case 30
                T(2,4) = T(2,4) + step;
            case 31
                T(2,4) = T(2,4) - step;
            case 43
                T(3,4) = T(3,4) + step;
            case 45
                T(3,4) = T(3,4) - step;
            case 97
                T(1:3,1:3) = T(1:3,1:3)*[1 0 0;0 cos(step_ang) -sin(step_ang);0  sin(step_ang) cos(step_ang)];
            case 122
                T(1:3,1:3) = T(1:3,1:3)*[1 0 0;0 cos(step_ang)  sin(step_ang);0 -sin(step_ang) cos(step_ang)];
            case 115
                T(1:3,1:3) = T(1:3,1:3)*[cos(step_ang) 0  sin(step_ang);0 1 0;-sin(step_ang) 0 cos(step_ang)];
            case 120
                T(1:3,1:3) = T(1:3,1:3)*[cos(step_ang) 0 -sin(step_ang);0 1 0; sin(step_ang) 0 cos(step_ang)];
            case 100
                T(1:3,1:3) = T(1:3,1:3)*[cos(step_ang) -sin(step_ang) 0; sin(step_ang) cos(step_ang) 0;0 0 1];
            case 99
                T(1:3,1:3) = T(1:3,1:3)*[cos(step_ang)  sin(step_ang) 0;-sin(step_ang) cos(step_ang) 0;0 0 1];
            case 109
                DT = datatip(smm_point_cloud);
                dcm_obj = datacursormode(fig);
                datacursormode on
                set(dcm_obj,'UpdateFcn',@moveDataTip_smm);
                while 1
                    if ~waitforbuttonpress
                        DT
                    else
                        if double(get(gcf,'CurrentCharacter'))==113
                            break
                        end
                    end
                end
            case 113 % lowercase q
                close all
                break
            case 81 % uppercase Q
                close
                break
            otherwise
                % Do nothing.
        end
    else
        % clc
    end

    %%% PLOT
    T
    qinv = inversekinematic(robot,T);
    q = qinv(:,solution_number);

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
        qs(1) = quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),0.1,'r');
        qs(2) = quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),0.1,'g');
        qs(3) = quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),0.1,'b');
        v = T(1:3,1:3)*u;
        qs(4) = quiver3(T(1,4),T(2,4),T(3,4),v(1)/norm(v),v(2)/norm(v),v(3)/norm(v),0.2,'k');
        hold off

    else

        disp('The current solution cannot reach this pose.')

    end
    %%% ENDPLOT

    title(strcat('Solution number: ',num2str(solution_number)))

    subplot(1,2,2);
    % Plot the eight solutions along self-motion manifolds.
    hold off
    smm = compute_smm(T,[0;1;1],N);
    smm_point_cloud = plot3(smm(:,1),smm(:,2),smm(:,6),'.b');
    hold on
    for sols=1:size(qinv,2)
        if isreal(qinv(:,sols))
            plot3(qinv(1,sols),qinv(2,sols),qinv(6,sols),'or');
        end
    end
    plot3(qinv(1,solution_number),qinv(2,solution_number),qinv(6,solution_number),'og','MarkerSize',10);
    grid on
    xlim([-pi,pi])
    ylim([-pi,pi])
    zlim([-pi,pi])
    
    ps
    qs
end

%%

function first_plot()

global robot T smm ps qs u fig

solution_number = 3;

qs = [];
ps = [];

N = 300;

%%% PLOT
    T
    qinv = inversekinematic(robot,T);
    q = qinv(:,solution_number);

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
        qs(1) = quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),0.1,'r');
        qs(2) = quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),0.1,'g');
        qs(3) = quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),0.1,'b');
        v = T(1:3,1:3)*u;
        qs(4) = quiver3(T(1,4),T(2,4),T(3,4),v(1)/norm(v),v(2)/norm(v),v(3)/norm(v),0.2,'k');
        hold off

    else

        disp('The current solution cannot reach this pose.')

    end
    %%% ENDPLOT

    title(strcat('Solution number: ',num2str(solution_number)))

    subplot(1,2,2);
    % Plot the eight solutions along self-motion manifolds.
    hold off
    smm = compute_smm(T,[0;1;1],N);
    smm_point_cloud = plot3(smm(:,1),smm(:,2),smm(:,6),'.b');
    hold on
    for sols=1:size(qinv,2)
        if isreal(qinv(:,sols))
            plot3(qinv(1,sols),qinv(2,sols),qinv(6,sols),'or');
        end
    end
    plot3(qinv(1,solution_number),qinv(2,solution_number),qinv(6,solution_number),'og','MarkerSize',10);
    grid on
    xlim([-pi,pi])
    ylim([-pi,pi])
    zlim([-pi,pi])

end