
close all
%compare the path planning back and forth
[qq_forth, manip_forth, index1_forth]=path_planning_max_manip_global('forth')
[qq_back, manip_back, index1_back]=path_planning_max_manip_global('back')

%Compare manipulability at each ponint
figure, plot(manip_forth)
hold
plot(manip_back(end:-1:1))
legend('manipulability forth', 'manipulability back')


figure, plot(qq_forth')
hold
plot(qq_back(:,end:-1:1)')
title('compare the two sets')