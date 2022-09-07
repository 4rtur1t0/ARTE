
%compare max, min a mean manipulability for vanilla
%load('/Users/arturogilaparicio/Desktop/arte/robots/RETHINK/sgo_v0.5/experiment1/mat/experiment1A.mat')

[correct_manips]=eval_experiment(Gout, random_manips);

[y,i]=max(sum(correct_manips'));
max_manip = correct_manips(i,:);

[y,i]=min(sum(correct_manips'));
min_manip = correct_manips(i,:);

mean_manip = mean(correct_manips);


figure, plot(max_manip), hold
plot(min_manip)
plot(mean_manip)
