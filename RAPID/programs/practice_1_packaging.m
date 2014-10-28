% Matlab script for RAPID equivalent commands
% please note that variables should be
% declared as global as in RAPID syntaxis
% this script demonstrates the use of functions in the matlab script. This
% script can be translated to RAPID by means of the matlab2RAPID script.
%
% IN ORDER TO SIMULATE THE PROGRAM:
%   A) FIRST, LOAD A ROBOT
%       >> robot = load_robot('abb','irb140');
%       OR
%       >> robot = load_robot('abb','irb52');
%   B) NEXT, LOAD SOME EQUIPMENT.
%       >> robot.equipment = load_robot('equipment','tables/table_two_areas');
%      
%   C) NOW, LOAD AN END TOOL
%       >> robot.tool= load_robot('equipment','end_tools/parallel_gripper_0');
%   D) FINALLY, LOAD A PIECE TO GRAB BY THE ROBOT
%       >> robot.piece=load_robot('equipment','cylinders/cylinder_tiny');
%
%   E) IF NECESSARY, CHANGE THE POSITION AND ORIENTATION OF THE piece,
%   relative to the robot's base reference system.
%
%       >> robot.piece.T0= [1 0 0 -0.1;
%                        0 1 0 -0.5;
%                        0 0 1 0.2;
%                        0 0 0 1]; 
%
%   The A, B, C, D and E steps can be performed usint the 'teach' application
%   by clicking on the Load equipment, load end tool, and load piece
%   buttons.
%
% during the simulation, call simulation_open_tool; to open the tool and 
% simulation_close_tool; to close it.
%
% To grip the piece, call simulation_grip_piece; and
% simulation_release_piece to release it.
%
% The call to each function must be in a correct, otherwise, the result may be
% undesired, thus, typically the correct sequence is:
% simulation_open_tool;
% (approach the piece to grab with MoveL or MoveJ)
% simulation_close_tool;
% simulation_grip_piece; --> the piece will be drawn with the robot
%
% move to a different place with MoveJ or MoveL
% simulation_open_tool;
% simulation_release_piece
%
% In RAPID, consider the use of ConfJ on, or ConfJ off, or ConfL on, or ConfL off
% in the case the controller avoids the change of configurations between target points
function practice_1_packaging

global RT_pos_ini RJ_ini RT_aprox_rec RT_pos_rec
global RT_aprox_dej RT_pos_dej TD_gripper VAR_pieza
global robot


TD_gripper=[1,[[0,0,0.125],[1,0,0,0]],[0.1,[0,0,0.100],[1,0,0,0],0,0,0]];

RJ_ini=[0,0,0,0,0,0];
RT_pos_ini=[[0.044,-0.501,0.432],[0.262525,0.622008,0.673812,-0.300276],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

RT_aprox_rec=[[0.03,-0.6,0.45],[0.08,0.60337,0.79283,-0.03126],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_pos_rec=[[0.0345,-0.629,0.337],[0.261833,0.652321,0.67413,-0.226869],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

RT_aprox_dej=[[0.470,-0.460,0.500],[0.07981,0.603204,0.792983,-0.030902],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
RT_pos_dej=[[0.470,-0.460,0.450],[0.07981,0.603204,0.792983,-0.030902],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


%local function to init simulation variables
init_simulation;


VAR_pieza=0;
%!Reset --> abrir pinza
%!Set --> Cerrar pinza

main
end


function main()

global RJ_ini TD_gripper VAR_pieza

%inicializar pieza
VAR_pieza=0;

MoveAbsJ(RJ_ini,'vmax','z100',TD_gripper, 'wobj0');

%MoveJ(RT_pos_ini,'vmax','z100',TD_gripper, 'wobj0');

for i=1:4,
    ASIR_DEJAR();
    VAR_pieza=VAR_pieza +1;
    %local function to init simulation variables
    init_simulation;
end

end


function ASIR_DEJAR()

global  RT_aprox_rec RT_aprox_dej TD_gripper 

%!Moverse a posici�n de aproximacion de recogida
MoveJ(RT_aprox_rec,'vmax','z100',TD_gripper, 'wobj0');
COGER_PIEZA;
%!Moverse a posici�n base de dejada
MoveJ(RT_aprox_dej,'vmax','z100',TD_gripper, 'wobj0');
METER_EN_CAJA;

end


function COGER_PIEZA()
global RT_aprox_rec RT_pos_rec TD_gripper

%!Ahora abrir pinza
simulation_open_tool; %Reset do1; 
WaitTime(0.1);!esperar apertura
%!Coger
MoveL(RT_pos_rec,'vmax','fine',TD_gripper, 'wobj0');

%!Ahora cerrar pinza
simulation_close_tool; %Set do1;
simulation_grip_piece;

Waittime(0.1); !esperar cierre
%!Subir por seguridad antes de ir al siguiente punto
MoveL(RT_aprox_rec,'vmax','z50',TD_gripper, 'wobj0');

end

function  METER_EN_CAJA()
global RT_pos_dej TD_gripper VAR_pieza

if VAR_pieza==0
    %Bajar para meter primera pieza
    MoveL(Offs(RT_pos_dej,0,0,-0.105),'v1000','fine',TD_gripper, 'wobj0');
elseif VAR_pieza==1
    MoveL(Offs(RT_pos_dej,-0.055,0,0),'v1000','z100',TD_gripper, 'wobj0');
    MoveL(Offs(RT_pos_dej,-0.055,0,-0.105),'v1000','fine',TD_gripper, 'wobj0');
elseif VAR_pieza==2
    MoveL(Offs(RT_pos_dej,0,-0.055,0),'v1000','fine',TD_gripper, 'wobj0');
    MoveL(Offs(RT_pos_dej,0,-0.055,-0.105),'v1000','fine',TD_gripper, 'wobj0');
else
    MoveL(Offs(RT_pos_dej,-0.055,-0.055,0),'v1000','fine',TD_gripper, 'wobj0');
    MoveL(Offs(RT_pos_dej,-0.055,-0.055,-0.105),'v1000','fine',TD_gripper, 'wobj0');
end

%Ahora abrir pinza
%yes, release the piece
simulation_open_tool; %Reset do1; 
simulation_release_piece; 

plot_points;

Waittime(0.5);
%Subir por seguridad antes de ir al siguiente punto
MoveL(Offs(RT_pos_dej,0,0,0.105),'v1000','z10',TD_gripper, 'wobj0');

end


function init_simulation
global robot
robot.piece.T0=eye(4);
robot.tool.Trel=eye(4);
% Now open the tool
simulation_open_tool; %Reset do1; 
simulation_release_piece;
%init the position of the piece at the beginning of the simulation
robot.piece.T0(1:3,4)=[0.03 -0.68 0.26]';
th=-30*pi/180;
u=[1 0 0;
   0 cos(th) -sin(th);
   0 sin(th) cos(th)];
robot.piece.T0(1:3,1:3)=u;%eye(3);
%robot.tool.piece_gripped=0;
drawrobot3d(robot, robot.q);


end

function plot_points
x0=0.47;
y0=-0.46;
z0=0.45-0.155;
delta=0.055;
X=[x0 x0-delta x0 x0-delta];
Y=[y0 y0 y0-delta y0-delta];
Z=[z0 z0 z0 z0];

plot3(X', Y', Z', 'LineWidth', 5)
end
