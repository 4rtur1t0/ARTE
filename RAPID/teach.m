function varargout = teach(varargin)
%
% TEACH MATLAB code for teach.fig
%      TEACH, by itself, creates a new TEACH or raises the existing
%      singleton*.
%
%      H = TEACH returns the handle to a new TEACH or the handle to
%      the existing singleton*.
%
%      TEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEACH.M with the given input arguments.
%
%      TEACH('Property','Value',...) creates a new TEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before teach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to teach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

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

% Edit the above text to modify the response to help teach

% Last Modified by GUIDE v2.5 07-Nov-2013 17:17:00
global configuration robot 
global controls program


if ~isfield(robot,'q') 
    warndlg('No robot variable found, please load a robot with: robot=load_robot','!! Warning !!')
    return;
end

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @teach_OpeningFcn, ...
                   'gui_OutputFcn',  @teach_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});    
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

  

% --- Executes just before teach is made visible.
function teach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to teach (see VARARGIN)

% Choose default command line output for teach
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes teach wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%initialize application variables
global robot configuration controls targets program

%reset program
program=[];
program.program_counter=0;
program.n_lines=0; %counter for the number of matlab lines

program.rapid_lines=[];
program.matlab_lines=[];

%reset targets
targets=[];

controls.num_target_points=0;
controls.global_gui_handle=hObject;


robot.q=zeros(1,robot.DOF)';

%reset position velocity acceleration and time
robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];
robot.time=[];


%update T and Q in the program
update_T_Q()
update_sliders();

figure(configuration.figure.robot);

disp('Select the desired view for your robot')
drawrobot3d(robot, robot.q);

% --- Outputs from this function are returned to the command line.
function varargout = teach_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu_instruction.
function popupmenu_instruction_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_instruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_instruction contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_instruction


% --- Executes during object creation, after setting all properties.
function popupmenu_instruction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_instruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls

controls.popupmenu_instruction=hObject;


% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_target_point.
function popupmenu_target_point_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_target_point contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_target_point


% --- Executes during object creation, after setting all properties.
function popupmenu_target_point_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls
controls.popupmenu_target_point=hObject;


% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_through_point.
function popupmenu_through_point_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_through_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_through_point contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_through_point


% --- Executes during object creation, after setting all properties.
function popupmenu_through_point_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_through_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.popupmenu_through_point=hObject;

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_speed.
function popupmenu_speed_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_speed contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_speed


% --- Executes during object creation, after setting all properties.
function popupmenu_speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls
controls.popupmenu_speed=hObject;

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu_precision.
function popupmenu_precision_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_precision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_precision contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_precision


% --- Executes during object creation, after setting all properties.
function popupmenu_precision_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_precision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.popupmenu_precision=hObject;
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_q1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q1, 'String', num2str(slider_value));
%convert to rads and store
if robot.kind(1)=='R'
    robot.q(1) = slider_value*pi/180;%rad
else
    robot.q(1) = slider_value;%m
end

update_T_Q();
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
draw_target_points();

% --- Executes during object creation, after setting all properties.
function slider_q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global robot controls
controls.slider_q1=hObject;


if robot.DOF>0
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(1,1)*180/pi,'Max',robot.maxangle(1,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    disp('\nPlease set robot.DOF correspondingly')
end

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on slider movement.
function slider_q2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q2, 'String', num2str(slider_value));

%convert to rads and store
if robot.kind(2)=='R'
    robot.q(2) = slider_value*pi/180;%rad
else
    robot.q(2) = slider_value;%m
end

update_T_Q();
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
draw_target_points();


% --- Executes during object creation, after setting all properties.
function slider_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global robot controls
controls.slider_q2=hObject;


if robot.DOF>1
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(2,1)*180/pi,'Max',robot.maxangle(2,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    %make the control disappear
    set(hObject, 'Min',0,'Max',0, 'Value',0);
    disp('\nPlease set robot.DOF correspondingly')
end

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_q3_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q3, 'String', num2str(slider_value));
%convert to rads and store
if robot.kind(3)=='R'
    robot.q(3) = slider_value*pi/180;%rad
else
    robot.q(3) = slider_value;%m
end
figure(configuration.figure.robot);
update_T_Q();
drawrobot3d(robot, robot.q);
draw_target_points();

% --- Executes during object creation, after setting all properties.
function slider_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global robot controls
controls.slider_q3=hObject;

if robot.DOF>2
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(3,1)*180/pi,'Max',robot.maxangle(3,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    %make the control disappear
     set(hObject, 'Min',0,'Max',0, 'Value',0);
    disp('\nPlease set robot.DOF correspondingly')
end

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_q4_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q4, 'String', num2str(slider_value));
%convert to rads and store
if robot.kind(4)=='R'
    robot.q(4) = slider_value*pi/180;%rad
else
    robot.q(4) = slider_value;%m
end

update_T_Q();
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
draw_target_points();

% --- Executes during object creation, after setting all properties.
function slider_q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global robot controls
controls.slider_q4=hObject;

if robot.DOF>3
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(4,1)*180/pi,'Max',robot.maxangle(4,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    %make the control disappear
     set(hObject, 'Min',0,'Max',0, 'Value',0);
    disp('\nPlease set robot.DOF correspondingly')
end


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_q5_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q5, 'String', num2str(slider_value));
%convert to rads and store
if robot.kind(5)=='R'
    robot.q(5) = slider_value*pi/180;%rad
else
    robot.q(5) = slider_value;%m
end
update_T_Q();
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
draw_target_points();

% --- Executes during object creation, after setting all properties.
function slider_q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global robot controls
controls.slider_q5=hObject;
if robot.DOF>4
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(5,1)*180/pi,'Max',robot.maxangle(5,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    %make the control disappear
     set(hObject, 'Min',0,'Max',0, 'Value',0);
    disp('\nPlease set robot.DOF correspondingly')
end


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_q6_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q6, 'String', num2str(slider_value));
%convert to rads and store
if robot.kind(6)=='R'
    robot.q(6) = slider_value*pi/180;%rad
else
    robot.q(6) = slider_value;%m
end
update_T_Q();
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
draw_target_points();


% --- Executes during object creation, after setting all properties.
function slider_q6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global robot controls

controls.slider_q6=hObject;

if robot.DOF>5
    %modify the limits of the slider object to fit max and min limits
    set(hObject, 'Min',robot.maxangle(6,1)*180/pi,'Max',robot.maxangle(6,2)*180/pi, 'Value',0,'SliderStep',[0.005 0.2]);
else 
    %make the control disappear
     set(hObject, 'Min',0,'Max',0, 'Value',0);
    disp('\nPlease set robot.DOF correspondingly')
end


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on button press in pushbutton_x_minus.
function pushbutton_x_minus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_x_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls


%FIND RESOLUTION OF MOVEMENT
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end



T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(1) = P_ini(1) - delta;
    else %move in end effector's coordinates
        P_final = P_ini - delta*T(1:3,1);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [1    0          0        0;
        0 cos(-delta) -sin(-delta) 0;
        0  sin(-delta) cos(-delta) 0;
        0   0              0     1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    tp=[[P_ini'],[Q],conf,ejes_ext];
    
    command =[' MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();


% --- Executes on button press in pushbutton_x_plus.
function pushbutton_x_plus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_x_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls
%substract this quantity to X in base coordinates
%compute total movement
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end


T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(1) = P_ini(1) + delta;
    else %move in end effector's coordinates
        P_final = P_ini + delta*T(1:3,1);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [1    0          0        0;
        0 cos(delta) -sin(delta) 0;
        0  sin(delta) cos(delta) 0;
        0   0              0     1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    tp=[[P_ini'],[Q],conf,ejes_ext];
    
    command =[' MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();


% --- Executes on button press in pushbutton_y_plus.
function pushbutton_y_plus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_y_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global robot controls
%substract this quantity to X in base coordinates
%compute total movement
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end


T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(2) = P_ini(2) + delta;
    else %move in end effector's coordinates
        P_final = P_ini + delta*T(1:3,2);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [cos(delta)    0          sin(delta)        0;
            0             1            0                0;
            -sin(delta)   0          cos(delta)         0;
            0             0              0               1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    tp=[[P_ini'],[Q],conf,ejes_ext];
    
    command =['MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();

% --- Executes on button press in pushbutton_y_minus.
function pushbutton_y_minus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_y_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot controls
%substract this quantity to X in base coordinates
%compute total movement
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end


T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(2) = P_ini(2) - delta;
    else %move in end effector's coordinates
        P_final = P_ini - delta*T(1:3,2);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [cos(-delta)    0          sin(-delta)        0;
            0             1            0                0;
            -sin(-delta)   0          cos(-delta)         0;
            0             0              0               1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    tp=[[P_ini'],[Q],conf,ejes_ext];
    
    command =['MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();

% --- Executes on button press in pushbutton_z_plus.
function pushbutton_z_plus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_z_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls
%substract this quantity to X in base coordinates

%compute total movement
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end


T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(3) = P_ini(3) + delta;
    else %move in end effector's coordinates
        P_final = P_ini + delta*T(1:3,3);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [cos(delta)       -sin(delta)          0     0;
            sin(delta)         cos(delta)         0     0;
                0                  0              1     0;
                0                   0              0    1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    %Do not remove tp, since it is evaluated
    tp=[[P_ini'],[Q],conf,ejes_ext];
    MoveJ(tp, 'vmax', 'fine', robot.tool0, robot.wobj0);
    %command =[' MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    %eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();

% --- Executes on button press in pushbutton_z_minus.
function pushbutton_z_minus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_z_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot controls
%substract this quantity to X in base coordinates
%compute total movement
resolution=get(controls.popupmenu_resolution,'Value');
if resolution==3 %low
    delta=0.05; %m
end
if resolution==2
    delta=0.1; %m
end
if resolution==1
    delta=0.2; %m
end


T = directkinematic(robot, robot.q);
P_ini=T(1:3,4);
% FIND KIND OF MOVEMENT, LINES OR REORIENTATION
line_reorient=get(controls.popupmenu_line_reorient,'Value');

% MOVE IN LINES 
if line_reorient==1
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        P_final = P_ini;
        P_final(3) = P_ini(3) - delta;
    else %move in end effector's coordinates
        P_final = P_ini - delta*T(1:3,3);
    end
    
    follow_line(P_ini,P_final);
    
else% REORIENTATION
    
    %compute objective T with rotation
    Trot = [cos(-delta)       -sin(-delta)          0     0;
            sin(-delta)         cos(-delta)         0     0;
                0                  0              1     0;
                0                   0              0    1];
    
    base=get(controls.popupmenu_base_end,'Value');
    
    if base==1 %move in base coordinates
        T_final = Trot*T;
    else %move in end effector's coordinates
        T_final = T*Trot;
    end
    
    ejes_ext = [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];
    conf=compute_configuration(robot, robot.q);
    
    
    %convert to quaternion
    Q = T2quaternion(T_final);
    Q=real(Q);
    tp=[[P_ini'],[Q],conf,ejes_ext];
    
    command =[' MoveJ(tp,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
    eval(command);
end

update_T_Q();
update_sliders();
draw_target_points();


function edit_q1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q1 as text
%        str2double(get(hObject,'String')) returns contents of edit_q1 as a double


% --- Executes during object creation, after setting all properties.
function edit_q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls

%save the control so that it can be accessed elsewhere
controls.edit_q1=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
    
end

%set(hObject, 'String', 'Push button pushed')

function edit_q2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q2 as text
%        str2double(get(hObject,'String')) returns contents of edit_q2 as a double


% --- Executes during object creation, after setting all properties.
function edit_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls

%save the control so that it can be accessed elsewhere
controls.edit_q2=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_q3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q3 as text
%        str2double(get(hObject,'String')) returns contents of edit_q3 as a double



% --- Executes during object creation, after setting all properties.
function edit_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls

%save the control so that it can be accessed elsewhere
controls.edit_q3=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_q4_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q4 as text
%        str2double(get(hObject,'String')) returns contents of edit_q4 as a double


% --- Executes during object creation, after setting all properties.
function edit_q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global controls

%save the control so that it can be accessed elsewhere
controls.edit_q4=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_q5_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q5 as text
%        str2double(get(hObject,'String')) returns contents of edit_q5 as a double


% --- Executes during object creation, after setting all properties.
function edit_q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global controls

%save the control so that it can be accessed elsewhere
controls.edit_q5=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_q6_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q6 as text
%        str2double(get(hObject,'String')) returns contents of edit_q6 as a double


% --- Executes during object creation, after setting all properties.
function edit_q6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


global controls

%save the control so that it can be accessed elsewhere
controls.edit_q6=hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_load.
function pushbutton_load_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global program controls targets robot

program=[];

%always require a filename
[filename, pathname] = uigetfile('*.m','Select .m program to load');
if isequal(filename,0)
   disp('User selected Cancel')
   return;
else
   disp(['User selected', fullfile(pathname, filename)])
end


%open selected file for read only
fp=fopen(fullfile(pathname, filename),'r');

disp('Loading target points');
while 1
    a = fgets(fp);
    k = strfind(a, '%BEGINTARGETPOINTS');
   
    %find beginning of target points
    if k>=0
        break;
    end
end

target_names=[];
%reset number of target points
N=0;
%now load target points
while 1
    a = fgets(fp);
    if length(a)<=1
        continue;
    end
    k = strfind(a, '%ENDTARGETPOINTS');
    %test whether there are no more target points
    if k>=0
        break;
    end
    
    %test whether its a valid target point initialization in matlab
    k = strfind(a, ':=');
    if k>=0
        continue;
    end

    [name, Q, T, conf]=get_targetpoint_from_string(a);
        
    N=N+1;
    targets{N}.name=name;
    targets{N}.Q=Q;
    targets{N}.T=T;
    targets{N}.conf=conf;
    
    
    qinv = inversekinematic(robot, T);
    q = select_configuration(robot, qinv, conf);
    targets{N}.q=q;
    
    target_names{N}=name;
    
    %store the name in the programming popup storing the available targets.
    set(controls.popupmenu_target_point,'String',target_names);  
    set(controls.popupmenu_through_point,'String',target_names); 
    
end
controls.num_target_points=N;

disp('Loading program');
program.n_lines=0;
%now load program
target_names=[];
%reset number of target points
N=0;
%now load target points
while 1
    a = fgets(fp);
    if length(a)<=1
        continue;
    end
    
    k = strfind(a, '%ENDMODULE');
    %test whether we reached the end of the program
    if k>=0
        break;
    end
    
    %test whether its a valid target point initialization in matlab
    k = strfind(a, '(');
    if isempty(k)
        continue;
    end

    N = N+1;
    
    program.n_lines=N;
    program.matlab_lines{N}=a(1:end-1); %remove carriage return    
    %program.rapid_lines{N}=a;
end

fclose(fp);

draw_target_points();


%Save the whole program to a .m file
% --- Executes on button press in pushbutton_save.
function pushbutton_save_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global program controls targets

%always require a filename
[filename, pathname] = uiputfile('*.m','Save as .m');
if isequal(filename,0) || isequal(pathname,0)
   disp('User selected Cancel')
   return;
else
   disp(['User selected',fullfile(pathname,filename)])
end


%open selected file
fp=fopen(fullfile(pathname, filename),'w');

disp('Saving target points');

fprintf(fp, '%%BEGINMODULE');
%write two escape characters to 
fprintf(fp, '\n%%BEGINTARGETPOINTS');
for i=1:controls.num_target_points,
    tp = construct_target(i);
    ejes_ext = '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];';
    a_rap = sprintf('%s:=[[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f], [%.5f, %.5f, %.5f, %.5f], %s',...
    targets{i}.name, targets{i}.T(1,4),targets{i}.T(2,4),targets{i}.T(3,4),...
            targets{i}.Q(1), targets{i}.Q(2), targets{i}.Q(3), targets{i}.Q(4),...
        targets{i}.conf(1), targets{i}.conf(2), targets{i}.conf(3), targets{i}.conf(4), ejes_ext);
    a_mat = sprintf('%s=[[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f], [%.5f, %.5f, %.5f, %.5f], %s',...
    targets{i}.name, targets{i}.T(1,4),targets{i}.T(2,4),targets{i}.T(3,4),...
            targets{i}.Q(1), targets{i}.Q(2), targets{i}.Q(3), targets{i}.Q(4),...
        targets{i}.conf(1), targets{i}.conf(2), targets{i}.conf(3), targets{i}.conf(4), ejes_ext);
    %write file    
    %fprintf(fp, '\n%%%s', a_rap);
    fprintf(fp, '\n%s', a_mat);
end

fprintf(fp, '\n%%ENDTARGETPOINTS\n');


disp('Saving program lines');
for i=1:program.n_lines,
    fprintf(fp, '\n%s', program.matlab_lines{i});
   % fprintf(fp, '\n%%%s', program.rapid_lines{i});
end

fprintf(fp, '\n%%ENDMODULE\n');
fclose(fp);


% simulate the whole program
% --- Executes on button press in pushbutton_simulate.
function pushbutton_simulate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_simulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global program targets controls robot

%reset position velocity acceleration and time
robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];
robot.time=[];


ejes_ext = '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];';

for i=1:controls.num_target_points,
    a_mat = sprintf('%s=[[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f], [%.5f, %.5f, %.5f, %.5f], %s',...
    targets{i}.name, targets{i}.T(1,4),targets{i}.T(2,4),targets{i}.T(3,4),...
    targets{i}.Q(1), targets{i}.Q(2), targets{i}.Q(3), targets{i}.Q(4),...
    targets{i}.conf(1), targets{i}.conf(2), targets{i}.conf(3), targets{i}.conf(4), ejes_ext);
    eval(a_mat);
end

for i=1:program.n_lines,
    eval(program.matlab_lines{i});
    update_T_Q();
    update_sliders();
    draw_target_points();
end

robot.q_vector=[];
robot.qd_vector=[];
robot.qdd_vector=[];
robot.time=[];



function edit_nx_Callback(hObject, eventdata, handles)
% hObject    handle to edit_nx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_nx as text
%        str2double(get(hObject,'String')) returns contents of edit_nx as a double


% --- Executes during object creation, after setting all properties.
function edit_nx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_nx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_nx=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ox_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_ox as text
%        str2double(get(hObject,'String')) returns contents of edit_ox as a double


% --- Executes during object creation, after setting all properties.
function edit_ox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_ox=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ax_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_ax as text
%        str2double(get(hObject,'String')) returns contents of edit_ax as a double


% --- Executes during object creation, after setting all properties.
function edit_ax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_ax=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pxt_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pxt as text
%        str2double(get(hObject,'String')) returns contents of edit_pxt as a double


% --- Executes during object creation, after setting all properties.
function edit_pxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pxt=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ny_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ny (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_ny as text
%        str2double(get(hObject,'String')) returns contents of edit_ny as a double


% --- Executes during object creation, after setting all properties.
function edit_ny_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ny (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_ny=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_oy_Callback(hObject, eventdata, handles)
% hObject    handle to edit_oy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_oy as text
%        str2double(get(hObject,'String')) returns contents of edit_oy as a double


% --- Executes during object creation, after setting all properties.
function edit_oy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_oy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_oy=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ay_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_ay as text
%        str2double(get(hObject,'String')) returns contents of edit_ay as a double


% --- Executes during object creation, after setting all properties.
function edit_ay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_ay=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pyt_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pyt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pyt as text
%        str2double(get(hObject,'String')) returns contents of edit_pyt as a double


% --- Executes during object creation, after setting all properties.
function edit_pyt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pyt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pyt=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_nz_Callback(hObject, eventdata, handles)
% hObject    handle to edit_nz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_nz as text
%        str2double(get(hObject,'String')) returns contents of edit_nz as a double


% --- Executes during object creation, after setting all properties.
function edit_nz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_nz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_nz=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_oz_Callback(hObject, eventdata, handles)
% hObject    handle to edit_oz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_oz as text
%        str2double(get(hObject,'String')) returns contents of edit_oz as a double


% --- Executes during object creation, after setting all properties.
function edit_oz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_oz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_oz=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_az_Callback(hObject, eventdata, handles)
% hObject    handle to edit_az (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_az as text
%        str2double(get(hObject,'String')) returns contents of edit_az as a double


% --- Executes during object creation, after setting all properties.
function edit_az_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_az (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_az=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pzt_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pzt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pzt as text
%        str2double(get(hObject,'String')) returns contents of edit_pzt as a double


% --- Executes during object creation, after setting all properties.
function edit_pzt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pzt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pzt=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% SOLVE THE INVERSE KINEMATIC PROBLEM AND GO TO IT
%   THE CLOSEST CONFIGURATION IS ALWAYS FOUND IN THE JOINT SPACE
% --- Executes on button press in pushbutton_moveT.
function pushbutton_moveT_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_moveT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global robot controls configuration

%get values of T from dialog
[T,Q,P]=get_TQ_from_dialog();

%if there is a tool attached to it, the total transformation has
% already considered it in the computation of direct kinematics
% reverse this transformation prior to the call to the inverse kinematics
% function
if isfield(robot, 'tool')
    %T=T*inv(robot.tool.TCP); 
    T=T/(robot.tool.TCP); 
end


%several solutions are provided
qinv = inversekinematic(robot, T);

distance_joint=[];
for i=1:size(qinv,2),
   distance_joint(i)=sum(abs(qinv(:,i)-robot.q(:))); 
end
[val,i]=min(distance_joint);

q_current=robot.q(:);
q_final = qinv(:,i);

path = lineal_path_plan(q_current, q_final);

robot.q_vector=path;
%Test whether there are joints outside mechanical limits
test_joint_limits(robot);

%do not move if current and final coordinates are the same
for i=1:size(path,2),
   drawrobot3d(robot, path(:,i)); 
   pause(configuration.time_delay);  
end

robot.q=q_final;
update_T_Q();
update_sliders();
draw_target_points()

function edit_Q0_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Q0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Q0 as text
%        str2double(get(hObject,'String')) returns contents of edit_Q0 as a double


% --- Executes during object creation, after setting all properties.
function edit_Q0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Q0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_Q0=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Q1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Q1 as text
%        str2double(get(hObject,'String')) returns contents of edit_Q1 as a double


% --- Executes during object creation, after setting all properties.
function edit_Q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_Q1=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Q2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Q2 as text
%        str2double(get(hObject,'String')) returns contents of edit_Q2 as a double


% --- Executes during object creation, after setting all properties.
function edit_Q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_Q2=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Q3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Q3 as text
%        str2double(get(hObject,'String')) returns contents of edit_Q3 as a double


% --- Executes during object creation, after setting all properties.
function edit_Q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_Q3=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pxq_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pxq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pxq as text
%        str2double(get(hObject,'String')) returns contents of edit_pxq as a double


% --- Executes during object creation, after setting all properties.
function edit_pxq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pxq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pxq=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pyq_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pyq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pyq as text
%        str2double(get(hObject,'String')) returns contents of edit_pyq as a double


% --- Executes during object creation, after setting all properties.
function edit_pyq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pyq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pyq=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pzq_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pzq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pzq as text
%        str2double(get(hObject,'String')) returns contents of edit_pzq as a double


% --- Executes during object creation, after setting all properties.
function edit_pzq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pzq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pzq=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



%when pressed, OBTAIN Q from dialog, convert to homogeneous matrix and
%plan a coordinated path to it
% --- Executes on button press in pushbutton_moveQ.
function pushbutton_moveQ_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_moveQ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global robot controls configuration

%get values of T from dialog
[T,Q,P]=get_TQ_from_dialog();

%convert quaternion to orientation
T = quaternion2T(Q);

T(1,4)=P(1);
T(2,4)=P(2);
T(3,4)=P(3);

%if there is a tool attached to it, the total transformation has
% already considered it in the computation of direct kinematics
% reverse this transformation prior to the call to the inverse kinematics
% function
if isfield(robot, 'tool')
    T=T*inv(robot.tool.TCP); 
end



%several solutions are provided
qinv = inversekinematic(robot, T);

distance_joint=[];
for i=1:size(qinv,2),
   distance_joint(i)=sum(abs(qinv(:,i)-robot.q(:))); 
end
[val,i]=min(distance_joint);

q_current=robot.q;
q_final = qinv(:,i);

path = lineal_path_plan(q_current, q_final);

robot.q_vector=path;
%Test whether there are joints outside mechanical limits
test_joint_limits(robot);

%do not move if current and final coordinates are the same
for i=1:size(path,2),
    drawrobot3d(robot, path(:,i)); 
    pause(configuration.time_delay);  
end

robot.q=q_final;
update_T_Q();
update_sliders();
draw_target_points()



%save a target point to memory
% --- Executes on button press in pushbutton_save_target_point.
function pushbutton_save_target_point_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global controls robot targets

target_point_name = get(controls.edit_target_point,'String');

%add a target point
controls.num_target_points = controls.num_target_points+1;

T = directkinematic(robot, robot.q);

%if there is a tool attached to it, consider it in the computation of 
% direct kinematics
if isfield(robot, 'tool')
    T=T*robot.tool.TCP; 
end

%convert to quaternion
Q = T2quaternion(T);
Q=real(Q);
target.T = T;
target.Q = Q;
target.q = robot.q;%store the current coordinates
target.conf = compute_configuration(robot, robot.q);
target.name = target_point_name;


targets{controls.num_target_points}=target;
draw_target_points();

s0=get(controls.popupmenu_target_point,'String');
a=length(s0);
s0{a+1}=target_point_name;

%store the name in the programming popup storing the available targets.
set(controls.popupmenu_target_point,'String',s0);


%the through point popupmenu
%s0=get(controls.popupmenu_through_point,'String');
%a=length(s0);
%s0{a+1}=target_point_name;

%store the name in the programming popup storing the available targets.
%set(controls.popupmenu_through_point,'String',s0);



function edit_target_point_Callback(hObject, eventdata, handles)
% hObject    handle to edit_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_target_point as text
%        str2double(get(hObject,'String')) returns contents of edit_target_point as a double


% --- Executes during object creation, after setting all properties.
function edit_target_point_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_target_point = hObject;

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_simulate_instruction.
% function pushbutton_simulate_instruction_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton_simulate_instruction (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% global controls targets robot
% 
% %reset position velocity acceleration and time
% robot.q_vector=[];
% robot.qd_vector=[];
% robot.qdd_vector=[];
% robot.time=[];
% 
% tool0=[];
% wobj0=[];
% 
% [instruction_matlab, instruction_rapid, tp1, tp2, joint_coord]=build_instruction_to_eval(); 
% eval(instruction_matlab);
% s = get(controls.popupmenu_instruction,'String');
% i = get(controls.popupmenu_instruction,'Value');
% s1=s{i};
% 
% switch s1
%     case {'MoveJ','MoveL'}
%         
%         s = get(controls.popupmenu_target_point,'String');
%         target_num = get(controls.popupmenu_target_point,'Value');
%         s2=s{target_num};
%         
%         s = get(controls.popupmenu_speed,'String');
%         i = get(controls.popupmenu_speed,'Value');
%         s3=s{i};
%         
%         s = get(controls.popupmenu_precision,'String');
%         i = get(controls.popupmenu_precision,'Value');
%         s4=s{i};
%         
%         instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
%         
%         tp=construct_target(target_num);
% 
%         instruction_matlab = ['robot=' s1 '(robot, tp, s3, s4, tool0, wobj0);'];
%         fprintf('\n**********************************************\n');
%         fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
%         fprintf('**********************************************\n');
%         
%         eval(instruction_matlab);
%         
%      case 'MoveAbsJ'
%         
%         s = get(controls.popupmenu_target_point,'String');
%         target_num = get(controls.popupmenu_target_point,'Value');
%         s2=s{target_num};
%         
%         s = get(controls.popupmenu_speed,'String');
%         i = get(controls.popupmenu_speed,'Value');
%         s3=s{i};
%         
%         s = get(controls.popupmenu_precision,'String');
%         i = get(controls.popupmenu_precision,'Value');
%         s4=s{i};
%         
%         instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
%         
%         
%         joint_coord=[targets{target_num}.q' 1e9, 1e9, 1e9, 1e9, 1e9, 1e9];
%         
%         instruction_matlab = ['robot=' s1 '(robot, joint_coord, s3, s4, tool0, wobj0);'];
%         fprintf('\n**********************************************\n');
%         fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
%         fprintf('**********************************************\n');
%         
%         eval(instruction_matlab);
%         
%         
%     case 'MoveC'
%         s = get(controls.popupmenu_target_point,'String');
%         target_num1 = get(controls.popupmenu_target_point,'Value');
%         s2=s{target_num1};
%         
%         s = get(controls.popupmenu_through_point,'String');
%         target_num2 = get(controls.popupmenu_through_point,'Value');
%         s3=s{target_num2};
%         
%         s = get(controls.popupmenu_speed,'String');
%         i = get(controls.popupmenu_speed,'Value');
%         vel=s{i};
%         
%         s = get(controls.popupmenu_precision,'String');
%         i = get(controls.popupmenu_precision,'Value');
%         prec=s{i};
%         
%         instruction_rapid = [s1 ' ' s3 ',' s2 ',' vel ',' prec  ',tool0\Wobj:=wobj0;'];
%         
%         tp2=construct_target(target_num1);
%         tp1=construct_target(target_num2);%the through point
%         
%         instruction_matlab = ['robot=' s1 '(robot, tp1, tp2, vel , prec);'];
%         fprintf('\n**********************************************\n');
%         fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
%         fprintf('**********************************************\n');
%         
%         eval(instruction_matlab);
%         
%     otherwise disp('RAPID instruction not supported');
% end

% %reset position velocity acceleration and time
% robot.q_vector=[];
% robot.qd_vector=[];
% robot.qdd_vector=[];
% robot.time=[];
% 
% update_T_Q();
% update_sliders();
% draw_target_points();

%clf(controls.global_gui_handle)
%drawnow()


% --- Executes on button press in pushbutton_save_instruction.
% function pushbutton_save_instruction_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton_save_instruction (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% global controls targets robot program

% %Get instruction name
% instruction = [];
% 
% s = get(controls.popupmenu_instruction,'String');
% i = get(controls.popupmenu_instruction,'Value');
% s1=s{i};
% 
% s = get(controls.popupmenu_target_point,'String');
% target_num = get(controls.popupmenu_target_point,'Value');
% s2=s{target_num};
% 
% s = get(controls.popupmenu_speed,'String');
% i = get(controls.popupmenu_speed,'Value');
% s3=s{i};
% 
% s = get(controls.popupmenu_precision,'String');
% i = get(controls.popupmenu_precision,'Value');
% s4=s{i};


% [instruction_matlab, instruction_rapid]=build_instruction_to_save(); 
% 
% 
% %program.program_counter = program.program_counter + 1;
% program.n_lines = program.n_lines + 1;
% 
% program.rapid_lines{program.n_lines} = instruction_rapid;%[s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
% program.matlab_lines{program.n_lines} = instruction_matlab;%['robot=' s1 '(robot,' s2 ',' sprintf('''%s''', s3) ',' sprintf('''%s''', s4) ');'];
% 
% 
% fprintf('\n**********************************************\n');
% fprintf('Adding instruction to program.\nCurrent program:');
% fprintf('\n**********************************************\n');
% for i=1:program.n_lines,
%     fprintf('\n%s', program.rapid_lines{i});
% end
% fprintf('\n**********************************************\n');
% update_T_Q();
% update_sliders();
% draw_target_points()





% --- Executes on key press with focus on slider_q1 and none of its controls.
function slider_q1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on selection change in popupmenu_base_end.
function popupmenu_base_end_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_base_end (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_base_end contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_base_end


% --- Executes during object creation, after setting all properties.
function popupmenu_base_end_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_base_end (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls
controls.popupmenu_base_end=hObject;

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%
%   REST OF FUNCTIONS TO HELP DIALOGS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%update the values from T to the dialog
function update_T_Q()

global robot controls


T = directkinematic(robot, robot.q);

%if there is a tool attached to it, consider it in the computation of 
% direct kinematics
if isfield(robot, 'tool')
    T=T*robot.tool.TCP; 
end

%Update T
set(controls.edit_nx, 'String', num2str(T(1,1), '%.3f'));
set(controls.edit_ny, 'String', num2str(T(2,1), '%.3f'));
set(controls.edit_nz, 'String', num2str(T(3,1), '%.3f'));
set(controls.edit_ox, 'String', num2str(T(1,2), '%.3f'));
set(controls.edit_oy, 'String', num2str(T(2,2), '%.3f'));
set(controls.edit_oz, 'String', num2str(T(3,2), '%.3f'));
set(controls.edit_ax, 'String', num2str(T(1,3), '%.3f'));
set(controls.edit_ay, 'String', num2str(T(2,3), '%.3f'));
set(controls.edit_az, 'String', num2str(T(3,3), '%.3f'));
set(controls.edit_pxt, 'String', num2str(T(1,4), '%.3f'));
set(controls.edit_pyt, 'String', num2str(T(2,4), '%.3f'));
set(controls.edit_pzt, 'String', num2str(T(3,4), '%.3f'));

%convert T to quaternion
Q = T2quaternion(T);
Q=real(Q);
set(controls.edit_Q0, 'String', num2str(Q(1),'%.3f'));
set(controls.edit_Q1, 'String', num2str(Q(2), '%.3f'));
set(controls.edit_Q2, 'String', num2str(Q(3), '%.3f'));
set(controls.edit_Q3, 'String', num2str(Q(4), '%.3f'));
%the same position as in T
set(controls.edit_pxq, 'String', num2str(T(1,4), '%.3f'));
set(controls.edit_pyq, 'String', num2str(T(2,4), '%.3f'));
set(controls.edit_pzq, 'String', num2str(T(3,4), '%.3f'));


%obtain the values of T Q and posittion from
function [T,Q,P]=get_TQ_from_dialog()

global robot controls


T = eye(4);

%Update T
T(1,1)=sscanf(get(controls.edit_nx, 'String'),'%f');
T(2,1)=sscanf(get(controls.edit_ny, 'String'), '%f');
T(3,1)=sscanf(get(controls.edit_nz, 'String'), '%f');
T(1,2)=sscanf(get(controls.edit_ox, 'String'), '%f');
T(2,2)=sscanf(get(controls.edit_oy, 'String'), '%f');
T(3,2)=sscanf(get(controls.edit_oz, 'String'), '%f');
T(1,3)=sscanf(get(controls.edit_ax, 'String'), '%f');
T(2,3)=sscanf(get(controls.edit_ay, 'String'), '%f');
T(3,3)=sscanf(get(controls.edit_az, 'String'), '%f');
T(1,4)=sscanf(get(controls.edit_pxt, 'String'), '%f');
T(2,4)=sscanf(get(controls.edit_pyt, 'String'), '%f');
T(3,4)=sscanf(get(controls.edit_pzt, 'String'), '%f');
 
Q = zeros(1,4);
Q(1)=sscanf(get(controls.edit_Q0, 'String'),'%f');
Q(2)=sscanf(get(controls.edit_Q1, 'String'),'%f');
Q(3)=sscanf(get(controls.edit_Q2, 'String'),'%f');
Q(4)=sscanf(get(controls.edit_Q3, 'String'),'%f');

P(1)=sscanf(get(controls.edit_pxq, 'String'), '%f');
P(2)=sscanf(get(controls.edit_pyq, 'String'), '%f');
P(3)=sscanf(get(controls.edit_pzq, 'String'), '%f');


%update the position of the sliders as well as the edit boxes associated to
%them
function update_sliders()

global robot controls

error = test_joints(robot, robot.q);

%do not update in case of any joint exceeds its range
if error == 1
    return;
end

if robot.DOF > 0
    %Update the sliders
    set(controls.slider_q1, 'Value', robot.q(1)*180/pi);
    %update the edit controls associated to them
    set(controls.edit_q1, 'String', num2str(robot.q(1)*180/pi,3));
end
if robot.DOF > 1
    set(controls.slider_q2, 'Value', robot.q(2)*180/pi);
    set(controls.edit_q2, 'String', num2str(robot.q(2)*180/pi,3));
end
if robot.DOF > 2
    set(controls.slider_q3, 'Value', robot.q(3)*180/pi);
    set(controls.edit_q3, 'String', num2str(robot.q(3)*180/pi,3));
end
if robot.DOF > 3
    set(controls.slider_q4, 'Value', robot.q(4)*180/pi);
    set(controls.edit_q4, 'String', num2str(robot.q(4)*180/pi,3));
end
if robot.DOF > 4
    set(controls.slider_q5, 'Value', robot.q(5)*180/pi);
    set(controls.edit_q5, 'String', num2str(robot.q(5)*180/pi,3));
end
if robot.DOF > 5
    set(controls.slider_q6, 'Value', robot.q(6)*180/pi);
    set(controls.edit_q6, 'String', num2str(robot.q(6)*180/pi,3));
end

if robot.DOF > 6
    disp('Not showing q when robot.DOF >= 7');
end


%plan a lineal path interpolating linearly the joint coordinates
function path=lineal_path_plan(q_ini, q_final)

global robot

%make num_points 
num_points = 15;

path = [];

for i=1:robot.DOF,
    delta = (q_final(i)-q_ini(i))/num_points;
    for j=1:num_points-1;
        path(i,j)= q_ini(i)+delta*j;
    end
    path(i,num_points)=q_final(i);
end



%plan a lineal path interpolating linearly the joint coordinates
function path=lineal_path_planXYZ(P_ini, P_final)

global robot

%make num_points 
num_points = 5;

path = [];

for i=1:3, %3, X, Y, Z
    delta = (P_final(i)-P_ini(i))/num_points;
    for j=1:num_points-1;
        path(i,j)= P_ini(i)+delta*j;
    end
    path(i,num_points)=P_final(i);
end



%follow a line in global coordinates
function follow_line(P_ini,P_final)
global robot configuration
%current end effector's position/orientation
T = directkinematic(robot, robot.q);

current_conf = compute_configuration(robot, robot.q);   

path = lineal_path_planXYZ(P_ini,P_final);

for i=1:size(path,2),
    T(1:3,4)=path(:,i);
    %several solutions are provided
    qinv = inversekinematic(robot, T); 
    %choose the closest to current position
   
    q=select_closest_joint_coordinates(qinv, robot.q);
    
    robot.q = q;
    
    %Test whether there are joints outside mechanical limits
    error = test_joints(robot, robot.q);
    
    drawrobot3d(robot, robot.q);
    
    %current_conf = compute_configuration(robot, robot.q);  
    
    if error == 1
        disp('\nAn error has occurred during the trajectory. Please check angle ranges');
        break;
    end
end
update_T_Q();
update_sliders();
plot3(path(1,:),path(2,:),path(3,:),'k', 'LineWidth', 3);

draw_target_points();



%draw the targets points on the robots figure
function draw_target_points()
global targets configuration controls
figure(configuration.figure.robot)

s=0.1; %scale

for i=1:controls.num_target_points,
    
    T = targets{i}.T;
    p0=T(1:3,4)';
    x=p0+s*T(1:3,1)';
    y=p0+s*T(1:3,2)';
    z=p0+s*T(1:3,3)';
    
    vect_arrow(p0,x,'r')
    vect_arrow(p0,y,'g')
    vect_arrow(p0,z,'b')
    %plot vector names X_i Y_i Z_i
    text(x(1)+0.01, x(2)+0.01, x(3)+0.01,targets{i}.name, 'FontWeight', 'bold', 'HorizontalAlignment', 'Center', 'FontSize', 14);
end


function tp = construct_target(target_num)

global targets

T = targets{target_num}.T;
Q = targets{target_num}.Q;
conf = targets{target_num}.conf;

tp = [[T(1,4),T(2,4),T(3,4)],[Q(1), Q(2), Q(3), Q(4)], [conf(1), conf(2), conf(3), conf(4)], [9E9,9E9,9E9,9E9,9E9,9E9]];



%decode a line into the different variables of a target point
function [name, Q, T, conf]=get_targetpoint_from_string(a)

%start decoding the line
[name, remain] = strtok(a,'=');
[token, remain] = strtok(remain,'[');

eval(['target=' remain]);

px=target(1);
py=target(2);
pz=target(3);

Q = target(4:7);
T = quaternion2T(Q, [px py pz]);

conf = target(8:11);




function [instruction_matlab, instruction_rapid, tp1, tp2, joint_coord]=build_instruction_to_eval() 

global controls targets robot

s = get(controls.popupmenu_instruction,'String');
i = get(controls.popupmenu_instruction,'Value');
s1=s{i};

tp1=[];
tp1_name=[];
tp2=[];
tp2_name = [];
joint_coord=[];
switch s1
    case {'MoveJ','MoveL'}
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        s3=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        s4=s{i};
        
        instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
        
        tp1=construct_target(target_num);
        
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        tp1_name=s{target_num};

        instruction_matlab = ['robot=' s1 '(robot, tp1,' sprintf(' ''%s'' ', s3) ',' sprintf(' ''%s'' ', s4) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
     case 'MoveAbsJ'
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        s3=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        s4=s{i};
        
        instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
        
        
        joint_coord=[targets{target_num}.q' 1e9, 1e9, 1e9, 1e9, 1e9, 1e9];
        
        
        instruction_matlab = ['robot=' s1 '(robot, joint_coord,' sprintf(' ''%s'' ', s3) ',' sprintf(' ''%s'' ', s4) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
        
    case 'MoveC'
        s = get(controls.popupmenu_target_point,'String');
        target_num1 = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num1};
        
        s = get(controls.popupmenu_through_point,'String');
        target_num2 = get(controls.popupmenu_through_point,'Value');
        s3=s{target_num2};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        vel=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        prec=s{i};
        
        instruction_rapid = [s1 ' ' s3 ',' s2 ',' vel ',' prec  ',tool0\Wobj:=wobj0;'];
        
        tp2=construct_target(target_num1);
        tp1=construct_target(target_num2);%the through point
        
        instruction_matlab = ['robot=' s1 '(robot, tp1, tp2, ' sprintf(' ''%s'' ', vel) ',' sprintf(' ''%s'' ', prec) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
    otherwise disp('RAPID instruction not supported');
end



function [instruction_matlab, instruction_rapid]=build_instruction_to_save() 

global controls targets robot

s = get(controls.popupmenu_instruction,'String');
i = get(controls.popupmenu_instruction,'Value');
s1=s{i};

tp1=[];
tp1_name=[];
tp2=[];
tp2_name = [];
joint_coord=[];
switch s1
    case {'MoveJ','MoveL'}
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        s3=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        s4=s{i};
        
        instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
        
              
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        tp_name=s{target_num};

        instruction_matlab = ['robot=' s1 '(robot,' tp_name ',' ...
            sprintf(' ''%s'' ', s3) ',' sprintf(' ''%s'' ', s4) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
     case 'MoveAbsJ'
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        s3=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        s4=s{i};
        
        instruction_rapid = [s1 ' ' s2 ',' s3 ',' s4 ',tool0\Wobj:=wobj0;'];
        
        
        %joint_coord=[targets{target_num}.q' 1e9, 1e9, 1e9, 1e9, 1e9, 1e9];
        
        s = get(controls.popupmenu_target_point,'String');
        target_num = get(controls.popupmenu_target_point,'Value');
        tp_name=s{target_num};
        
        instruction_matlab = ['robot=' s1 '(robot,' tp_name ','...
            sprintf(' ''%s'' ', s3) ',' sprintf(' ''%s'' ', s4) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
        
    case 'MoveC'
        s = get(controls.popupmenu_target_point,'String');
        target_num1 = get(controls.popupmenu_target_point,'Value');
        s2=s{target_num1};
        
        s = get(controls.popupmenu_through_point,'String');
        target_num2 = get(controls.popupmenu_through_point,'Value');
        s3=s{target_num2};
        
        s = get(controls.popupmenu_speed,'String');
        i = get(controls.popupmenu_speed,'Value');
        vel=s{i};
        
        s = get(controls.popupmenu_precision,'String');
        i = get(controls.popupmenu_precision,'Value');
        prec=s{i};
        
        instruction_rapid = [s1 ' ' s3 ',' s2 ',' vel ',' prec  ',tool0\Wobj:=wobj0;'];
        
        
        
        instruction_matlab = ['robot=' s1 '(robot,' s3 ',' s2 ', ' sprintf(' ''%s'' ', vel) ',' sprintf(' ''%s'' ', prec) ', robot.tool0, robot.wobj0);'];
        fprintf('\n**********************************************\n');
        fprintf('Evaluating instruction:\nRAPID:\n\t%s\nMatlab:\n\t%s\n', instruction_rapid, instruction_matlab);
        fprintf('**********************************************\n');
        
       % eval(instruction_matlab);
        
    otherwise disp('RAPID instruction not supported');
end


% --- Executes on selection change in popupmenu_line_reorient.
function popupmenu_line_reorient_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_line_reorient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_line_reorient contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_line_reorient


% --- Executes during object creation, after setting all properties.
function popupmenu_line_reorient_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_line_reorient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


global controls
controls.popupmenu_line_reorient=hObject;



% SHOWS CURRENT TARGET POINT IN RAPID on the command line
% A rounding towards the 4 decimal point is performed, in order to make the
% target points more readable. Rounding errors may cause differences in the
% computed configurations from the original point. Please be aware that the
% configuration corresponds to the point and orientation written in the
% point.
%
% --- Executes on button press in pushbutton_show_target_point.
function pushbutton_show_target_point_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_show_target_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global controls targets robot

T = directkinematic(robot, robot.q);

%if there is a tool attached to it, consider it in the computation of 
% direct kinematics
if isfield(robot, 'tool')
    T=T*robot.tool.TCP; 
    disp('Show Target Point: The tool has been considered in the Target Point! OK!')
end

%convert to quaternion
Q = T2quaternion(T);
%avoid imaginari parts
Q = real(Q);
conf = compute_configuration(robot, robot.q);

%the target points written to the command line are rounded to a 4 digit
%precision. This may cause discrepancies in the cf1, cf4 or cf6 variables
%(the configuration in RAPID). Thus, the points are first rounded, next,
%the inversekinematic is computed for the rounded points the closest
%solution
tp=round_4_digits(T, Q, conf);

fprintf('\n\ntp:=[[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f,%.4f],[%d,%d,%d,%d],[9E9,9E9,9E9,9E9,9E9,9E9]];\n\n',tp(1),tp(2),tp(3),tp(4),tp(5),tp(6),tp(7),tp(8),tp(9),tp(10),tp(11));


function tp=round_4_digits(T, Q, conf)

global robot
%tp is rounded to the 4 digits 
tp = [round(10000*[T(1,4),T(2,4),T(3,4)])/10000,round(10000*[Q(1), Q(2), Q(3), Q(4)])/10000, [conf(1), conf(2), conf(3), conf(4)], [9E9,9E9,9E9,9E9,9E9,9E9]];

%Compute the transformation for the rounded target point
T_target = transform_to_homogeneous(tp);

%if there is a tool attached to it, consider it in the computation of 
% inverse kinematics
if isfield(robot, 'tool')
    T = T_target/robot.tool.TCP;
end

q=inversekinematic(robot, T_target);

% s=[];
% %now, find the closest q to the current joint values robot.q
% for i=1:robot.DOF,
%     s=[s sum(abs(q(:,i)-robot.q))];
% end
% [val,i]=min(s);

index=find_closest(q, robot.q);

%now, q corresponds to the joint values that correspond to the rounded
%target point
T = directkinematic(robot, q(:,index));

%convert to quaternion
Q = T2quaternion(T);
%avoid imaginari parts
Q = real(Q);
conf = compute_configuration(robot, q(:,index));

tp = [[T(1,4),T(2,4),T(3,4)],[Q(1), Q(2), Q(3), Q(4)], [conf(1), conf(2), conf(3), conf(4)], [9E9,9E9,9E9,9E9,9E9,9E9]];



% --- Executes on key press with focus on pushbutton_show_target_point and none of its controls.
% SHOWS THE CURRENT TP WHERE THE ROBOT IS SITUATED
function pushbutton_show_target_point_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_show_target_point (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)





% --- Executes on selection change in popupmenu_resolution.
function popupmenu_resolution_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_resolution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_resolution contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_resolution


% --- Executes during object creation, after setting all properties.
function popupmenu_resolution_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_resolution (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

global controls
controls.popupmenu_resolution=hObject;


% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RESETS THE ROBOTS CURRENT JOINTS TO 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in pushbutton_reset_all.
function pushbutton_reset_all_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset_all (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global configuration robot controls

update_T_Q();
figure(configuration.figure.robot);


drawrobot3d(robot, robot.q);

final_joints = zeros(1,robot.DOF);
%Use MoveAbsJ to move to zero coordinates
command =[' MoveAbsJ(final_joints,' sprintf(' ''%s'' ', 'vmax') ',' sprintf(' ''%s'' ', 'fine') ',robot.tool0, robot.wobj0);'];
eval(command);
robot.q=final_joints;

update_T_Q();
draw_target_points();
update_sliders();


% --- Executes on key press with focus on pushbutton_save_target_point and none of its controls.
function pushbutton_save_target_point_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_save_target_point (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)








function index=find_closest(q,qi)

qq=zeros(size(q,1),size(q,2));
for i=1:size(q,2),
    qq(:,i)=qi;
end

a=sum((q-qq).^2);

[val, index] = min(a);


% --- Executes on button press in pushbutton_load_robot.
function pushbutton_load_robot_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global configuration robot controls

%
robot=load_robot()

% --- Executes on key press with focus on pushbutton_load_robot and none of its controls.
function pushbutton_load_robot_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_load_robot (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in push_button_load_end_tool.
function push_button_load_end_tool_Callback(hObject, eventdata, handles)
% hObject    handle to push_button_load_end_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global configuration robot controls
%

robot.tool=load_robot

% --- Executes on key press with focus on push_button_load_end_tool and none of its controls.
function push_button_load_end_tool_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to push_button_load_end_tool (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on button press in push_button_load_environment.
function push_button_load_environment_Callback(hObject, eventdata, handles)
% hObject    handle to push_button_load_environment (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global configuration robot controls
%
robot.equipment=load_robot

% --- Executes on key press with focus on push_button_load_environment and none of its controls.
function push_button_load_environment_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to push_button_load_environment (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on pushbutton_simulate_instruction and none of its controls.
function pushbutton_simulate_instruction_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_simulate_instruction (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in push_button_save_TP.
function push_button_save_TP_Callback(hObject, eventdata, handles)
% hObject    handle to push_button_save_TP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global controls targets robot


%always require a filename
[filename, pathname] = uiputfile('*.m','Save as .m');
if isequal(filename,0) || isequal(pathname,0)
   disp('User selected Cancel')
   return;
else
   disp(['User selected',fullfile(pathname,filename)])
end


%open selected file
fp=fopen(fullfile(pathname, filename),'w');

fprintf('Saving %d target points to file', controls.num_target_points);

for i=1:controls.num_target_points,
    %tp = construct_target(i);
  
    tp=round_4_digits(targets{i}.T, targets{i}.Q, targets{i}.conf);
    ejes_ext = '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];';
    %a_rap = sprintf('%s:=[[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f], [%.5f, %.5f, %.5f, %.5f], %s',...
%     targets{i}.name, targets{i}.T(1,4),targets{i}.T(2,4),targets{i}.T(3,4),...
%             targets{i}.Q(1), targets{i}.Q(2), targets{i}.Q(3), targets{i}.Q(4),...
%         targets{i}.conf(1), targets{i}.conf(2), targets{i}.conf(3), targets{i}.conf(4), ejes_ext);
%     a_mat = sprintf('%s=[[%.4f, %.4f, %.4f],[%.4f, %.4f, %.4f, %.4f], [%d, %d, %d, %d], %s',...
%     targets{i}.name, targets{i}.T(1,4),targets{i}.T(2,4),targets{i}.T(3,4),...
%             targets{i}.Q(1), targets{i}.Q(2), targets{i}.Q(3), targets{i}.Q(4),...
%         targets{i}.conf(1), targets{i}.conf(2), targets{i}.conf(3), targets{i}.conf(4), ejes_ext);
    a_mat = sprintf('%s=[[%.4f, %.4f, %.4f],[%.4f, %.4f, %.4f, %.4f], [%d, %d, %d, %d], %s',...
    targets{i}.name, tp(1), tp(2), tp(3),... %name, position
            tp(4), tp(5), tp(6), tp(7),... %quaternion
        tp(8), tp(9),tp(10), tp(11), ejes_ext); %configuration
    %write file    
    %fprintf(fp, '\n%%%s', a_rap);
    fprintf(fp, '\n%s', a_mat);
end

fclose(fp);


% --- Executes on button press in pushbutton_refresh.
function pushbutton_refresh_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_refresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot

update_T_Q();
update_sliders();
draw_target_points();
drawrobot3d(robot, robot.q)

% --- Executes on key press with focus on pushbutton_refresh and none of its controls.
function pushbutton_refresh_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_refresh (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on button press in pushbutton_load_piece.
function pushbutton_load_piece_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load_piece (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot

robot.piece=load_robot();%('equipment/cylinders/cylinder_tiny');
%robot.piece.T0(1:3,4)=[0 -0.45 0.2]';
