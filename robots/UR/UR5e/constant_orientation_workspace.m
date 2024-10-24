global T robot fig fig2 ps qinv solution_number

solution_number = 4;

ps = [];

T = directkinematic(robot,zeros(1,6)); % Home pose
R = T(1:3,1:3); % Home orientation
ang = 0.5;
R = R*[cos(pi/6) 0 sin(pi/6);0 1 0;-sin(pi/6) 0 cos(pi/6)];
R = R*[1 0 0;0 cos(ang) -sin(ang);0 sin(ang) cos(ang)];
T(1:3,1:3) = R;
T(2,4) = -0.5;

x_min = -1;
x_max =  1;
z_min = -1;
z_max =  1.2;

Nx = 200;
dx = (x_max-x_min)/(Nx-1);
Nz = 200;
dz = (z_max-z_min)/(Nz-1);

COWS = zeros(Nx*Nz,3);
counter = 1;

x = x_min;
for i=1:Nx
    z = z_min;
    for j=1:Nz
        number_of_reals = 0;
        T(1,4) = x;
        T(3,4) = z;
        qinv = inversekinematic(robot,T);
        reachable = 0;
        for solutions = 1:size(qinv,2)
            is_real = 1;
            for joints = 1:6
                is_real = isreal( qinv(joints,solutions) ) && is_real;
                if ~is_real
                    break
                end
            end
            number_of_reals = number_of_reals + is_real;
        end
        if number_of_reals > 0
            COWS(counter,:) = [x,z,number_of_reals];
            counter = counter + 1;
        end
        z = z + dz;
    end
    x = x + dx;
end

COWS = COWS(1:counter-1,:);

%%

solution_number = 1;

close all
fig = figure(1)

% drawrobot3d(robot,zeros(1,6))
light
grid on
daspect([1 1 1])

hold on
% plot3(COWS(:,1),COWS(:,1)*0+T(2,4),COWS(:,2),'.r')
S = scatter3(COWS(:,1), COWS(:,1)*0+T(2,4), COWS(:,2), [], COWS(:,3), 'filled', 'o')
S.SizeData = 10;
xlim([-1,1])

dcm_obj = datacursormode(fig);
datacursormode on
set(dcm_obj,'UpdateFcn',@moveDataTip);

% fig2 = figure(2)
% dcm_obj2 = datacursormode(fig2);
% datacursormode on
% set(dcm_obj2,'UpdateFcn',@moveDataTip2);






