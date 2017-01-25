function varargout = PP_forGUI(varargin)
% PP_FORGUI MATLAB code for PP_forGUI.fig
%      PP_FORGUI, by itself, creates a new PP_FORGUI or raises the existing
%      singleton*.
%
%      H = PP_FORGUI returns the handle to a new PP_FORGUI or the handle to
%      the existing singleton*.
%
%      PP_FORGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PP_FORGUI.M with the given input arguments.
%
%      PP_FORGUI('Property','Value',...) creates a new PP_FORGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PP_forGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PP_forGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PP_forGUI

% Last Modified by GUIDE v2.5 30-Dec-2016 12:32:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @PP_forGUI_OpeningFcn, ...
    'gui_OutputFcn',  @PP_forGUI_OutputFcn, ...
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


% --- Executes just before PP_forGUI is made visible.
function PP_forGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PP_forGUI (see VARARGIN)

% Choose default command line output for PP_forGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PP_forGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PP_forGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_start.
function pushbutton_start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global P_radom P_start
P_start = P_radom(1,:);
plot(P_start(1),P_start(2),'mo')
set(handles.edit_display,'String','Set start point!')


% --- Executes on button press in pushbutton_goal.
function pushbutton_goal_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global P_radom P_goal
P_goal = P_radom(2,:);
plot(P_goal(1),P_goal(2),'ko')
set(handles.edit_display,'String','Set goal point!')


function edit_random_number_Callback(hObject, eventdata, handles)
% hObject    handle to edit_random_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_random_number as text
%        str2double(get(hObject,'String')) returns contents of edit_random_number as a double


% --- Executes during object creation, after setting all properties.
function edit_random_number_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_random_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_plan.
function pushbutton_plan_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_plan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global P_start P_goal P_obstacles algorithm_choosing Angle_goal_ 

num_mov_ob = str2double(get(handles.edit_moving_obstacles_number,'String'));

switch algorithm_choosing
    case 1
        set(handles.edit_display,'String','Please choose an algorithm!');
    case 2
        set(handles.edit_display,'String',['Planning with APF!',num_mov_ob]);
        [flag_apf, step_in_all] = APF_main_forGUI(P_start, P_goal, P_obstacles, Angle_goal_, num_mov_ob);        
        if flag_apf
            set(handles.edit_display,'String',['planning success!',' And total steps is : ', num2str(step_in_all)])
        end
    case 3
        %[Q_trees_,D_start_new] = RRT_random(P_start, P_goal, P_obstacles)
        RRT_main_forGUI(P_start, P_goal, P_obstacles, Angle_goal_, num_mov_ob)
        set(handles.edit_display,'String','Planning with RRT!');
end

% --- Executes on button press in pushbutton_clear.
function pushbutton_clear_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla
set(handles.edit_display,'String','Clear the figure!')

% --- Executes on button press in pushbutton_obstacles.
function pushbutton_obstacles_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_obstacles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global P_radom P_obstacles
obstacle_num = size(P_radom,1)-2;
if obstacle_num
    P_obstacles = P_radom(3:end,:);
    plot(P_obstacles(:,1),P_obstacles(:,2),'bo')
    ob_str = ['Set ',num2str(obstacle_num),' obstacles!'];
    set(handles.edit_display,'String',ob_str)
else
    set(handles.edit_display,'String','No obstacles setting!')
end

function edit_display_Callback(hObject, eventdata, handles)
% hObject    handle to edit_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_display as text
%        str2double(get(hObject,'String')) returns contents of edit_display as a double


% --- Executes during object creation, after setting all properties.
function edit_display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_moving_obstacles_number_Callback(hObject, eventdata, handles)
% hObject    handle to edit_moving_obstacles_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_moving_obstacles_number as text
%        str2double(get(hObject,'String')) returns contents of edit_moving_obstacles_number as a double


% --- Executes during object creation, after setting all properties.
function edit_moving_obstacles_number_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_moving_obstacles_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton_moving_goal.
function radiobutton_moving_goal_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_moving_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton_moving_goal
global Angle_goal_ 
flag_Movinggoal = get(handles.radiobutton_moving_goal,'value');
if (flag_Movinggoal)
    Angle_goal_ = randi([-180,180])*pi/180;
    set(handles.edit_display,'String','The goal is moving.');
else
    Angle_goal_ = [];
    set(handles.edit_display,'String','The goal is not moving.');
end


% --- Executes on button press in pushbutton_random.
function pushbutton_random_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global P_radom random_num xy_range
format long
xy_range = 10;
random_num = str2double(get(handles.edit_random_number,'String'));
r_str = ['Set ',num2str(random_num),' random points!'];
set(handles.edit_display,'String',r_str)
P_radom = xy_range*rand(random_num,2);
plot(P_radom(:,1),P_radom(:,2),'go')
hold on
axis([-1 xy_range+1 -1 xy_range+1])

% --- Executes on selection change in popupmenu_algorithms.
function popupmenu_algorithms_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_algorithms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_algorithms contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_algorithms
global algorithm_choosing
algorithm_choosing = get(handles.popupmenu_algorithms,'Value');

switch algorithm_choosing
    case 1
        set(handles.edit_display,'String','Please choose an algorithm!');
    case 2
        set(handles.edit_display,'String','Choose APF algorithm!');
    case 3
        set(handles.edit_display,'String','Choose RRT algorithm!');
end

% --- Executes during object creation, after setting all properties.
function popupmenu_algorithms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_algorithms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
