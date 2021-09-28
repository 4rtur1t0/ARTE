
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%experiment from min comparison%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('experimento_from_min_vanilla.mat')
%7172
figure, plot(pathq'), hold
w_from_min_vanilla = compute_manip(robot, pathq);
load('experimento_from_min_sgo.mat')
plot(pathq')

w_from_min_sgo = compute_manip(robot, pathq);

n1 = length(w_from_min_vanilla)
n2 = length(w_from_min_sgo)
n = min(n1, n2);

w_from_min_vanilla=w_from_min_vanilla(1:n);
w_from_min_sgo=w_from_min_sgo(1:n);
figure, plot(w_from_min_vanilla)
hold
plot(w_from_min_sgo)
legend('from_min_vanilla', 'from_min_sgo')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%experiment from max comparison%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('experimento_from_max_vanilla.mat')
%7172
%figure, plot(pathq'), hold
w_from_max_vanilla = compute_manip(robot, pathq);
load('experimento_from_max_sgo.mat')
%plot(pathq')
w_from_max_sgo = compute_manip(robot, pathq);

n1 = length(w_from_max_vanilla)
n2 = length(w_from_max_sgo)
n = min(n1, n2);

w_from_max_vanilla=w_from_max_vanilla(1:n);
w_from_max_sgo=w_from_max_sgo(1:n);
figure, plot(w_from_max_vanilla)
hold
plot(w_from_max_sgo)
legend('from_max_vanilla', 'from_max_sgo')