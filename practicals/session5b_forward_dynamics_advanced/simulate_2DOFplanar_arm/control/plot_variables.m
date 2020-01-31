% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
close all

figure, plot(tout, control_results)
title('Joint positions')

figure, 
plot(tout, joint1_error), hold
plot(tout, joint2_error)
title('Joint errors')


figure, 
plot(tout, motor1_current), hold
plot(tout, motor2_current)
title('Motor current')



figure, 
plot(tout, motor1_speed_rpm), hold
plot(tout, motor2_speed_rpm)
title('Motor speed rpm')
