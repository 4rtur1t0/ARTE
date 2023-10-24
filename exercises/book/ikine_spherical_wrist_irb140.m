% A SYMBOLIC DERIVATION OF THE JACOBIAN MANIPULATOR OF A KUKA LBR IIWA 14
%
% Copyright (C) 2016, by Arturo Gil Aparicio
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
function ikine_spherical_wrist_irb140
% link lengths



R = Rot(pi/4, 'x')*Rot(pi/4, 'y')*Rot(pi/4, 'z');
[w1, w2, w3, w4] = ikine_wrist(R)
 
 Q=check(w1);
 R-Q
 Q=check(w2);
 R-Q
 Q=check(w3);
 R-Q
 Q=check(w4);
 R-Q

 function [w1, w2, w3, w4] = ikine_wrist(Q)
     
 thresh = 0.0001;   
 if 1 - abs(Q(3, 3)) > thresh
     q5 = acos(Q(3, 3))  
     s5 = sign(q5)          
     q4 = atan2(-s5 * Q(2, 3), -s5 * Q(1, 3))
     q6 = atan2(s5 * Q(3, 2), -s5 * Q(3, 1))
     
     q5_ = -q5     
     s5_ = sign(q5_)     
     q4_ = atan2(-s5_ * Q(2, 3), -s5_ * Q(1, 3))     
     q6_ = atan2(s5_ * Q(3, 2), -s5_ * Q(3, 1)) 
 else
     q5 = np.real(acos(Q(3, 3)))
     q5_ = q5
     q4 = 0
     q4_ = np.pi
     q6 = atan2(Q(1, 2), -Q(2, 2))
     q6_ = q6 - np.pi
 end
        
 w1 = [q4, q5, q6];
 w2 = [q4_, q5_, q6_];
 w3 = [q4+2*pi, q5, q6+2*pi];
 w4 = [q4_-2*pi, q5_, q6_-2*pi];
 
function Q = check(w)
q4 = w(1);
q5 = w(2);
q6 = w(3);
Q = [sin(q4)*sin(q6) - cos(q4)*cos(q5)*cos(q6), cos(q6)*sin(q4) + cos(q4)*cos(q5)*sin(q6), -cos(q4)*sin(q5);
 - cos(q4)*sin(q6) - cos(q5)*cos(q6)*sin(q4), cos(q5)*sin(q4)*sin(q6) - cos(q4)*cos(q6), -sin(q4)*sin(q5);
                            -cos(q6)*sin(q5),                           sin(q5)*sin(q6),          cos(q5)];
        
