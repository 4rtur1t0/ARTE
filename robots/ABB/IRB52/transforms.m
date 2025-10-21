

robot = load_robot('ABB', 'IRB52');
A01 = dh(robot, [0 0 0 0 0 0], 1)
A12 = dh(robot, [0 0 0 0 0 0], 2)
A23 = dh(robot, [0 0 0 0 0 0], 3)
A34 = dh(robot, [0 0 0 0 0 0], 4)
A45 = dh(robot, [0 0 0 0 0 0], 5)
A56 = dh(robot, [0 0 0 0 0 0], 6)

% PLACE Z1 AXIS (q2) AND link
A01
rot2euler(A01, 'XYZ')
% PLACE FIRST AXIS AND link
A02 = A01*A12
rot2euler(A02, 'XYZ')
A03 = A01*A12*A23
rot2euler(A03, 'XYZ')
A04 = A01*A12*A23*A34
rot2euler(A04, 'XYZ')
A05 = A01*A12*A23*A34*A45
rot2euler(A05, 'XYZ')
A06 = A01*A12*A23*A34*A45*A56
rot2euler(A06, 'XYZ')









