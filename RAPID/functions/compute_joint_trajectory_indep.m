%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   [qt, qdt, qddt] = compute_joint_trajectory(q_ini, q_final, time_vector, qd_ini, qd_final)
%   
%   Computes an smooth trajectory between the joint coordinates Q_ini and
%   Q_final using a 5th degree polynomial. 
%   A polynomial for each joint is computed and evaluated for the proposed
%   time vectord taking into account the inputs:
%   
%   Q_ini, Q_final: The initial a final joint coordinates q_ini and q_final
%   time_vector: The time vector when the movement is to be planned.
%   Qd_ini, Qd_final: The initial and final speeds of each joint.
%
%   Outputs: 
%   Qt: the smooth joint trajectory computed by the function at each time
%   step.
%   Qdt: the joint speed at each time step.
%   Qddt: the joint acceleration at each time step.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche
%   Date: 05/05/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
function [qt, qdt, qddt] = compute_joint_trajectory_indep(robot, q_ini, q_final, qd_ini, qd_final, time_vector)

%n=robot.DOF;

qt=zeros(1,length(time_vector));%[];
qdt=zeros(1,length(time_vector));
qddt=zeros(1,length(time_vector));

for i=1:robot.DOF,
    [qi, qdi, qddi, poly]=single_joint_spline(q_ini(i), q_final(i), qd_ini(i), qd_final(i), time_vector);
    qt(i,:)=qi;
    qdt(i,:)=qdi;
    qddt(i,:)=qddi;    
end
