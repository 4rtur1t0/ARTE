%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   q = solve_spherical_wrist(robot, q, T, wrist)	
%   Solves the inverse kinematic problem for a spherical wrist
%   robot: robot structure.
%   q: vector containing the values of the joints 1, 2 and 3.
%   T: orientation of the last reference system.
%   wrist: select -1 or 1 for two possible solutions (wrist up, wrist down)
%   
%	See also DIRECTKINEMATIC.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

function q = solve_spherical_wrist(robot, q, T, wrist, method)

switch method
    
     %algebraic solution
    case 'algebraic'
        T01=dh(robot, q, 1);
        T12=dh(robot, q, 2);
        T23=dh(robot, q, 3);
        
        Q=inv(T23)*inv(T12)*inv(T01)*T;
        
        %detect the degenerate case when q(5)=0, this leads to zeros
        % in Q13, Q23, Q31 and Q32 and Q33=1
        thresh=1e-12;
        %detect if q(5)==0
        % this happens when cos(q5) in the matrix Q is close to 1
        if abs(Q(3,3)-1)>thresh 
            %normal solution
            if wrist==1 %wrist up
                q(4)=atan2(-Q(2,3),-Q(1,3));        
                q(6)=atan2(-Q(3,2),Q(3,1));            
                %q(5)=atan2(-Q(3,2)/sin(q(6)),Q(3,3));
            else %wrist down
                q(4)=atan2(-Q(2,3),-Q(1,3))-pi;            
                q(6)=atan2(-Q(3,2),Q(3,1))+pi;            
                %q(5)=atan2(-Q(3,2)/sin(q(6)),Q(3,3));
            end
            if abs(cos(q(6)+q(4)))>thresh 
                cq5=(Q(1,1)+Q(2,2))/cos(q(4)+q(6))-1;
            end
            if abs(sin(q(6)+q(4)))>thresh
                cq5=(-Q(1,2)+Q(2,1))/sin(q(4)+q(6))-1;
            end
            if abs(sin(q(6)))>thresh
                sq5=-Q(3,2)/sin(q(6));
            end
            if abs(cos(q(6)))>thresh
                sq5=Q(3,1)/cos(q(6));
            end
            q(5)=atan2(sq5,cq5);
            
        else %degenerate solution, in this case, q4 cannot be determined,
             % so q(4)=0 is assigned
            if wrist==1 %wrist up
                q(4)=0;
                q(5)=0;
                q(6)=atan2(-Q(1,2)+Q(2,1),Q(1,1)+Q(2,2));
            else %wrist down
                q(4)=-pi;
                q(5)=0;
                q(6)=atan2(-Q(1,2)+Q(2,1),Q(1,1)+Q(2,2))+pi;
            end             
           
        end  
 
       %geometric solution 
    case 'geometric' 
        % T is the noa matrix defining the position/orientation of the end
        % effector's reference system
        vx6=T(1:3,1);
        vz5=T(1:3,3); % The vector a z6=T(1:3,3) is coincident with z5
        
        % Obtain the position and orientation of the system 3
        % using the already computed joints q1, q2 and q3
        T01=dh(robot, q, 1);
        T12=dh(robot, q, 2);
        T23=dh(robot, q, 3);
        T03=T01*T12*T23;
         
        vx3=T03(1:3,1);
        vy3=T03(1:3,2);
        vz3=T03(1:3,3);
        
        % find z4 normal to the plane formed by z3 and a
        vz4=cross(vz3, vz5);	% end effector's vector a: T(1:3,3)
        
        % in case of degenerate solution,
        % when vz3 and vz6 are parallel--> then z4=0 0 0, choose q(4)=0 as solution
        if norm(vz4) <= 0.000001
            if wrist == 1 %wrist up
                q(4)=0;
            else
                q(4)=-pi; %wrist down
            end
        else
            %this is the normal and most frequent solution
            cosq4=wrist*dot(-vy3,vz4);
            sinq4=wrist*dot(vx3,vz4);
            q(4)=atan2(sinq4, cosq4);
        end
        %propagate the value of q(4) to compute the system 4
        T34=dh(robot, q, 4);
        T04=T03*T34;
        vx4=T04(1:3,1);
        vy4=T04(1:3,2);
             
        % solve for q5 
        cosq5=dot(vy4,vz5);
        sinq5=dot(-vx4,vz5);
        q(5)=atan2(sinq5, cosq5);
        
        %propagate now q(5) to compute T05
        T45=dh(robot, q, 5);
        T05=T04*T45;
        vx5=T05(1:3,1);
        vy5=T05(1:3,2);
        
        % solve for q6
        cosq6=dot(vx6,vx5);
        sinq6=dot(vx6,vy5);
        q(6)=atan2(sinq6, cosq6);     
        
    
        
    otherwise
        disp('no method specified in solve_spherical_wrist');
end
