N = 50;
q1 = 0:pi/4/(N-1):pi/4;
q2 = 0:(-pi/3)/(N-1):-pi/3;
q3 = 0:(-0.15)/(N-1):(-0.15);
q4 = 0:pi/(N-1):pi;

for i=1:N
    drawrobot3d(robot,[q1(i),q2(i),q3(i),q4(i)])
    pause(0.01);
end