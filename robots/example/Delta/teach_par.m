function varargout = teach_par(varargin)
% TEACH_PAR M-file for teach_par.fig
%      TEACH_PAR, by itself, creates a new TEACH_PAR or raises the existing
%      singleton*.
%
%      H = TEACH_PAR returns the handle to a new TEACH_PAR or the handle to
%      the existing singleton*.
%
%      TEACH_PAR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEACH_PAR.M with the given input arguments.
%
%      TEACH_PAR('Property','Value',...) creates a new TEACH_PAR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before teach_par_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to teach_par_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help teach_par

% Last Modified by GUIDE v2.5 16-Dec-2013 12:14:25
global configuration robot 
global controls program

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @teach_par_OpeningFcn, ...
                   'gui_OutputFcn',  @teach_par_OutputFcn, ...
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


% --- Executes just before teach_par is made visible.
function teach_par_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to teach_par (see VARARGIN)

global robot

T0=[1 0 0 0; 0 1 0 0; 0 0 1 -0.4; 0 0 0 1];
robot.q=inversekinematic(robot,T0);
drawrobot3d(robot,robot.q);
update_sliders();
update_T_Q();

% Choose default command line output for teach_par
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes teach_par wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = teach_par_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_q1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of
%        slider
global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q1, 'String', num2str(slider_value));
%convert to rads and store
robot.q(1) = slider_value*pi/180;%rad

update_T_Q();
T=directkinematic(robot,robot.q);
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
% draw_target_points();
% --- Executes during object creation, after setting all properties.


function slider_q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global robot controls
controls.slider_q1=hObject;

%Maybe for separate the robots by the number of arms
% if robot.nserial > 0
%     set(hObject, 'Min',0,'Max',2*pi, 'Value',0,'SliderStep',[0.005 0.2]);
% else
%     disp('\nPlease set robot.nserial correspondly')
% end

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider_q2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global configuration robot controls
slider_value = get(hObject,'Value');
set(controls.edit_q2, 'String', num2str(slider_value));
%convert to rads and store
robot.q(4) = slider_value*pi/180%rad

update_T_Q();
T=directkinematic(robot,robot.q);
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
% draw_target_points();

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global robot controls
controls.slider_q2=hObject;

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
robot.q(7) = slider_value*pi/180;%rad

update_T_Q();
T=directkinematic(robot,robot.q);
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
% draw_target_points();


% --- Executes during object creation, after setting all properties.
function slider_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global robot controls
controls.slider_q3=hObject;

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
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
%        str2double(get(hObject,'String')) returns contents of edit_q3 as a
%        double

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


% --- Executes on button press in X_plus.
function X_plus_Callback(hObject, eventdata, handles)
% hObject    handle to X_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(1,4)=T(1,4) + delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q)

update_sliders();
update_T_Q()

% --- Executes on button press in X_minus.
function X_minus_Callback(hObject, eventdata, handles)
% hObject    handle to X_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(1,4)=T(1,4) - delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
update_sliders();
update_T_Q()

% --- Executes on button press in Y_plus.
function Y_plus_Callback(hObject, eventdata, handles)
% hObject    handle to Y_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(2,4)=T(2,4) + delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q)
update_sliders();
update_T_Q()

% --- Executes on button press in Y_minus.
function Y_minus_Callback(hObject, eventdata, handles)
% hObject    handle to Y_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(2,4)=T(2,4) - delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);
update_sliders();
update_T_Q()

% --- Executes on button press in Z_plus.
function Z_plus_Callback(hObject, eventdata, handles)
% hObject    handle to Z_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(3,4)=T(3,4) + delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q)
update_sliders();
update_T_Q()
% --- Executes on button press in Z_minus.
function Z_minus_Callback(hObject, eventdata, handles)
% hObject    handle to Z_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot controls configuration

delta = 0.1;
T=directkinematic(robot,robot.q);
T(3,4)=T(3,4) - delta;
robot.q=inversekinematic(robot,T);
figure(configuration.figure.robot);
drawrobot3d(robot, robot.q);update_sliders();
update_T_Q()



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



function edit_px_Callback(hObject, eventdata, handles)
% hObject    handle to edit_px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_px as text
%        str2double(get(hObject,'String')) returns contents of edit_px as a double


% --- Executes during object creation, after setting all properties.
function edit_px_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_px=hObject;
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



function edit_py_Callback(hObject, eventdata, handles)
% hObject    handle to edit_py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_py as text
%        str2double(get(hObject,'String')) returns contents of edit_py as a double


% --- Executes during object creation, after setting all properties.
function edit_py_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_py=hObject;
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



function edit_pz_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pz as text
%        str2double(get(hObject,'String')) returns contents of edit_pz as a double


% --- Executes during object creation, after setting all properties.
function edit_pz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_pz=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_MoveT.
function pushbutton_MoveT_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_MoveT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global robot controls configuration

%get values of T from dialog
[T,Q,P]=get_TQ_from_dialog();


robot.q=inversekinematic(robot,T);
drawrobot3d(robot,robot.q);
update_sliders();

% --- Executes during object creation, after setting all properties.
function pushbutton_MoveT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_MoveT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit_QT0_Callback(hObject, eventdata, handles)
% hObject    handle to edit_QT0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_QT0 as text
%        str2double(get(hObject,'String')) returns contents of edit_QT0 as a double


% --- Executes during object creation, after setting all properties.
function edit_QT0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_QT0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_QT0=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_QT1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_QT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_QT1 as text
%        str2double(get(hObject,'String')) returns contents of edit_QT1 as a double


% --- Executes during object creation, after setting all properties.
function edit_QT1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_QT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_QT1=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_QT2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_QT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_QT2 as text
%        str2double(get(hObject,'String')) returns contents of edit_QT2 as a double


% --- Executes during object creation, after setting all properties.
function edit_QT2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_QT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_QT2=hObject;
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function edit_QT3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_QT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_QT3 as text
%        str2double(get(hObject,'String')) returns contents of edit_QT3 as a double


% --- Executes during object creation, after setting all properties.
function edit_QT3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_QT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global controls
controls.edit_QT3=hObject;
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

robot.q = inversekinematic(robot, T);
drawrobot3d(robot,robot.q);
update_T_Q();
update_sliders();

% --- Executes during object creation, after setting all properties.
function pushbutton_moveQ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_moveQ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function update_T_Q()

global robot controls

T = directkinematic(robot, robot.q);

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
set(controls.edit_px, 'String', num2str(T(1,4), '%.3f'));
set(controls.edit_py, 'String', num2str(T(2,4), '%.3f'));
set(controls.edit_pz, 'String', num2str(T(3,4), '%.3f'));

%convert T to quaternion
Q = T2quaternion(T);
Q=real(Q);
set(controls.edit_QT0, 'String', num2str(Q(1),'%.3f'));
set(controls.edit_QT1, 'String', num2str(Q(2), '%.3f'));
set(controls.edit_QT2, 'String', num2str(Q(3), '%.3f'));
set(controls.edit_QT3, 'String', num2str(Q(3), '%.3f'));
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
T(1,4)=sscanf(get(controls.edit_px, 'String'), '%f');
T(2,4)=sscanf(get(controls.edit_py, 'String'), '%f');
T(3,4)=sscanf(get(controls.edit_pz, 'String'), '%f');
 
Q = zeros(1,4);
Q(1)=sscanf(get(controls.edit_QT0, 'String'),'%f');
Q(2)=sscanf(get(controls.edit_QT1, 'String'),'%f');
Q(3)=sscanf(get(controls.edit_QT2, 'String'),'%f');
Q(4)=sscanf(get(controls.edit_QT3, 'String'),'%f');

P(1)=sscanf(get(controls.edit_pxq, 'String'), '%f');
P(2)=sscanf(get(controls.edit_pyq, 'String'), '%f');
P(3)=sscanf(get(controls.edit_pzq, 'String'), '%f');


function update_sliders()

global robot controls

% error = test_joints(robot, robot.q);
% 
% %do not update in case of any joint exceeds its range
% if error == 1
% return;
% end

%Update the sliders
set(controls.slider_q1, 'Value', robot.q(1)*180/pi);
set(controls.slider_q2, 'Value', robot.q(4)*180/pi);
set(controls.slider_q3, 'Value', robot.q(7)*180/pi);


%update the edit controls associated to them
set(controls.edit_q1, 'String', num2str(robot.q(1)*180/pi,3));
set(controls.edit_q2, 'String', num2str(robot.q(4)*180/pi,3));
set(controls.edit_q3, 'String', num2str(robot.q(7)*180/pi,3));


% --- Executes on button press in pushbutton_resetQ.
function pushbutton_resetQ_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_resetQ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global configuration robot

T0=[1 0 0 0; 0 1 0 0; 0 0 1 -0.4; 0 0 0 1];
robot.q=inversekinematic(robot,T0);
drawrobot3d(robot,robot.q);
update_sliders();
update_T_Q();


% --- Executes during object creation, after setting all properties.
function pushbutton_resetQ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_resetQ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
