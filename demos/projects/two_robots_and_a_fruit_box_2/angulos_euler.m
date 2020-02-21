function [p]=angulos_euler(robot,q)

T=directkinematic(robot,q);
betha=asin(T(1,3));
gamma=asin(-T(1,2)/cos(betha));
alpha=acos(T(3,3)/cos(betha));
p=[T(1,4) T(2,4) T(3,4) alpha betha gamma];

end