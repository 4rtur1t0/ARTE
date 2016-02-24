% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       robot = load_robot('abb','irb140');
%       OR
%       robot = load_robot('abb','irb52');
%       or any other robot, the target points may not be reachable,
%       depending on the links' lengths.
%
%   B) NEXT, LOAD SOME EQUIPMENT.
%       robot.equipment = load_robot('equipment','tables/table_two_areas');
%      
%   
%
% The example presents the robot achieving different positions and orientations
% In addition, the conf values, change accordingly.


function test_configurations2

global  TD_tool0 robot

%definition of the end effector.
TD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];



main
end

function main
global  TD_tool0 robot


MoveJ([[217.4,1.57,1072.81]/1000,[0.972982,-0.01947,0.224767,-0.049073],[0,1,-3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[86.96,1.57,1072.8]/1000,[0.972992,-0.01944,0.224739,-0.049009],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[28.15,1.55,1072.8]/1000,[0.973021,-0.019611,0.224611,-0.048949],[1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[14.42,1.55,1072.8]/1000,[0.973015,-0.019655,0.224611,-0.049041],[1,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[0.11,1.57,1072.82]/1000,[0.973031,-0.019796,0.224522,-0.049091],[1,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[-12.28,1.55,1072.81]/1000,[0.973022,-0.019733,0.224521,-0.049298],[1,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[-98.02,1.54,1072.82]/1000,[0.973023,-0.019651,0.22448,-0.049486],[1,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
MoveJ( [[-231.93,1.54,1072.81]/1000,[0.973047,-0.019566,0.224335,-0.049716],[1,-2,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],'vmax','z50',TD_tool0);
  

end