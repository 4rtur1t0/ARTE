function varargout = coord_change(varargin)
% COORD_CHANGE MATLAB code for coord_change.fig
%       COORD_CHANGE is a graphical assistant that helps you define each
%       stl file in coordinates of its own reference system.
%
%       a) Start by selecting a .stl file, for example belonging to link0 in terms 
%       of the base reference system. 
%       b) To change units from mm to meters. Input this T:
%               T = [1 0 0 0;
%                    0 1 0 0;
%                    0 0 1 0;
%                    0 0 0 1000]
%       c) After defining T click on "Transform by T". Next click on "View
%       link".
%       d) Change T so that the axis with respect to the link are
%       coincident with the D-H reference system attached to the current
%       link.
%       e) When you are finished, just save the current link as link0.stl,
%       link1.stl, link2.stl, depending on the link you are working on.
%       f) Whenever the 6 links have been processed, you can load the robot
%       with "Load Robot" and draw it with "Draw robot".
%
%
%
%
%      COORD_CHANGE, by itself, creates a new COORD_CHANGE or raises the existing
%      singleton*.
%
%      H = COORD_CHANGE returns the handle to a new COORD_CHANGE or the handle to
%      the existing singleton*.
%
%      COORD_CHANGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in COORD_CHANGE.M with the given input arguments.
%
%      COORD_CHANGE('Property','Value',...) creates a new COORD_CHANGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before coord_change_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to coord_change_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help coord_change

% Last Modified by GUIDE v2.5 27-Oct-2016 17:32:06
global robot 
global current_link
global fout_global
global vout_global
global cout_global
%global figure_handle

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @coord_change_OpeningFcn, ...
                   'gui_OutputFcn',  @coord_change_OutputFcn, ...
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


% --- Executes just before coord_change is made visible.
function coord_change_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to coord_change (see VARARGIN)

%global figure_handle
%figure_handle = figure
% Choose default command line output for coord_change
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes coord_change wait for user response (see UIRESUME)
% uiwait(handles.figure1);

T = eye(4);

%Update T
set(handles.edit_nx, 'String', num2str(T(1,1), '%.3f'));
set(handles.edit_ny, 'String', num2str(T(2,1), '%.3f'));
set(handles.edit_nz, 'String', num2str(T(3,1), '%.3f'));
set(handles.edit_ox, 'String', num2str(T(1,2), '%.3f'));
set(handles.edit_oy, 'String', num2str(T(2,2), '%.3f'));
set(handles.edit_oz, 'String', num2str(T(3,2), '%.3f'));
set(handles.edit_ax, 'String', num2str(T(1,3), '%.3f'));
set(handles.edit_ay, 'String', num2str(T(2,3), '%.3f'));
set(handles.edit_az, 'String', num2str(T(3,3), '%.3f'));
set(handles.edit_px, 'String', num2str(T(1,4), '%.3f'));
set(handles.edit_py, 'String', num2str(T(2,4), '%.3f'));
set(handles.edit_pz, 'String', num2str(T(3,4), '%.3f'));
set(handles.edit_T_scale, 'String', num2str(T(4,4), '%.3f'));


% --- Outputs from this function are returned to the command line.
function varargout = coord_change_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_load_robot.
function pushbutton_load_robot_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot
robot = load_robot


% --- Executes on button press in pushbutton_draw_robot.
function pushbutton_draw_robot_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_draw_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot
drawrobot3d(robot, [0 0 0 0 0 0])


% --- Executes on button press in pushbutton_load_link.
function pushbutton_load_link_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load_link (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fout_global
global vout_global
global cout_global

[FileName,PathName,FilterIndex] = uigetfile({'*.stl','Link files (*.stl)'},'Pick the link stl file in base coordinates.', 'MultiSelect', 'off');
full_name=[PathName '/' FileName];
cd(PathName);

%file_i=sprintf('/link%d.stl', i);
%read file in base reference system. Please note that link0 is already
%defined in the base reference system. In the first loop T= identity
fprintf('\nReading link in BASE REFERNCE COORDINATES\n %s\n', full_name);
[fout_global, vout_global, cout_global] = stl_read(full_name);
    


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

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_transform_by.
function pushbutton_transform_by_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_transform_by (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fout_global
global vout_global
global cout_global

T = get_T_from_dialog(handles);
%change points from mm to meters.
V=vout_global/T(4,4);
V(:,4) = ones(length(V),1); %homogeneous coordinates
%inverse transform, the points V in link i are now referred to its own D-H reference system
V = (inv(T)*V')';
vout_global  = V(:,1:3);


% --- Executes on button press in pushbutton_view_link.
function pushbutton_view_link_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_view_link (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vout_global
global cout_global
global fout_global
%global figure_handle
T = get_T_from_dialog(handles);

%hold off
figure
hold on
grid

%obtain link points
V=vout_global;
color=[255 102 51]./255
%set robot.graphical.color to add a desired color to your robot
draw_patch(fout_global,vout_global,color, 1);
draw_axes(eye(4), 'X', 'Y','Z', 1);


% --- Executes on button press in pushbutton_save_link_as.
function pushbutton_save_link_as_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save_link_as (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fout_global
global vout_global

[FileName,PathName,FilterIndex] = uiputfile({'*.stl','Link files (*.stl)'},'Save link stl file as');
full_name=[PathName '/' FileName];
stlwrite(full_name, fout_global, vout_global, 'mode', 'ascii');


%obtain the values of T Q and posittion from
function [T]=get_T_from_dialog(handles)

%init as identity matrix
T = eye(4);

%Update T
T(1,1)=sscanf(get(handles.edit_nx, 'String'),'%f');
T(2,1)=sscanf(get(handles.edit_ny, 'String'), '%f');
T(3,1)=sscanf(get(handles.edit_nz, 'String'), '%f');
T(1,2)=sscanf(get(handles.edit_ox, 'String'), '%f');
T(2,2)=sscanf(get(handles.edit_oy, 'String'), '%f');
T(3,2)=sscanf(get(handles.edit_oz, 'String'), '%f');
T(1,3)=sscanf(get(handles.edit_ax, 'String'), '%f');
T(2,3)=sscanf(get(handles.edit_ay, 'String'), '%f');
T(3,3)=sscanf(get(handles.edit_az, 'String'), '%f');
T(1,4)=sscanf(get(handles.edit_px, 'String'), '%f');
T(2,4)=sscanf(get(handles.edit_py, 'String'), '%f');
T(3,4)=sscanf(get(handles.edit_pz, 'String'), '%f');
T(4,4)=sscanf(get(handles.edit_T_scale, 'String'), '%f');



function edit_T_scale_Callback(hObject, eventdata, handles)
% hObject    handle to edit_T_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_T_scale as text
%        str2double(get(hObject,'String')) returns contents of edit_T_scale as a double


% --- Executes during object creation, after setting all properties.
function edit_T_scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_T_scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_reset_T.
function pushbutton_reset_T_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset_T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

T = eye(4);

%Update T
set(handles.edit_nx, 'String', num2str(T(1,1), '%.3f'));
set(handles.edit_ny, 'String', num2str(T(2,1), '%.3f'));
set(handles.edit_nz, 'String', num2str(T(3,1), '%.3f'));
set(handles.edit_ox, 'String', num2str(T(1,2), '%.3f'));
set(handles.edit_oy, 'String', num2str(T(2,2), '%.3f'));
set(handles.edit_oz, 'String', num2str(T(3,2), '%.3f'));
set(handles.edit_ax, 'String', num2str(T(1,3), '%.3f'));
set(handles.edit_ay, 'String', num2str(T(2,3), '%.3f'));
set(handles.edit_az, 'String', num2str(T(3,3), '%.3f'));
set(handles.edit_px, 'String', num2str(T(1,4), '%.3f'));
set(handles.edit_py, 'String', num2str(T(2,4), '%.3f'));
set(handles.edit_pz, 'String', num2str(T(3,4), '%.3f'));
set(handles.edit_T_scale, 'String', num2str(T(4,4), '%.3f'));


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

help coord_change
