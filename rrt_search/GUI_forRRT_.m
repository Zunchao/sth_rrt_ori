function varargout = GUI_forRRT_(varargin)
% GUI_FORRRT_ MATLAB code for GUI_forRRT_.fig

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GUI_forRRT__OpeningFcn, ...
    'gui_OutputFcn',  @GUI_forRRT__OutputFcn, ...
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


% --- Executes just before GUI_forRRT_ is made visible.
function GUI_forRRT__OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_forRRT__OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

% --- Executes on button press in pushbutton_RUN_MAIN.
function pushbutton_RUN_MAIN_Callback(hObject, eventdata, handles)
global P_start P_goal N_obstacles algorithm_choosing
N_obstacles = str2double(get(handles.edit_ob,'String'));

switch algorithm_choosing
    case 1
        set(handles.edit_DISPLAY,'String','Please choose an algorithm!');
    case 2
        cla
        hold on
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run naive rrt ...')
        RRT_forGUI_naive(P_start, P_goal);
    case 3
        cla
        hold on
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run rrt random ...')
        RRT_forGUI_random(P_start, P_goal)
        set(handles.edit_DISPLAY,'String','Planning with RRT!');
    case 4
        cla
        hold on
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run rrt by voronoi ...')
        RRT_forGUI_voronoi(P_start, P_goal)
        
    case 5
        cla
        %plot(P_start(1,1),P_start(1,2),'ro')
        hold on
        %plot(P_goal(1,1)-1.5,P_goal(1,2)-1.5,'ko')
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run rrt connected ...')
        RRT_forGUI_random_connect(P_start, P_goal)
    case 6
        cla
        plot(P_start(1,1),P_start(1,2),'ro')
        hold on
        plot(P_goal(1,1),P_goal(1,2),'ko')
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run rrt merged ...')
        RRT_forGUI_random_merge(P_start, P_goal)
    case 7
        cla
        hold on
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        
        disp('run rrt with circular obstacles ...')
        if ~N_obstacles
            set(handles.edit_DISPLAY,'String','Please set the num of obstacles > 0 !');
        else
            RRT_forGUI_random_with_ob(P_goal, N_obstacles)
        end
    case 8
        cla
        hold on
        axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
        disp('run rrt with cube obstacles ...')
        if ~N_obstacles
            set(handles.edit_DISPLAY,'String','Please set the num of obstacles > 0 !');
        else
            RRT_forGUI_random_with_ob_cube(P_start, P_goal, N_obstacles)
        end
end

% --- Executes on selection change in popupmenu_algorithms.
function popupmenu_algorithms_Callback(hObject, eventdata, handles)
global algorithm_choosing
algorithm_choosing = get(handles.popupmenu_algorithms,'Value');

switch algorithm_choosing
    case 1
        set(handles.edit_DISPLAY,'String','Please choose an algorithm!');
    case 2
        set(handles.edit_DISPLAY,'String','Choose naive rrt!');
    case 3
        set(handles.edit_DISPLAY,'String','Choose rrt by voronoi!');
    case 4
        set(handles.edit_DISPLAY,'String','Choose run rrt random!');
    case 5
        set(handles.edit_DISPLAY,'String','Choose rrt connected!');
    case 6
        set(handles.edit_DISPLAY,'String','Choose rrt merged!');
    case 7
        set(handles.edit_DISPLAY,'String','Choose rrt with circular obstacles!');
    case 8
        set(handles.edit_DISPLAY,'String','Choose rrt with cube obstacles!');
end

% --- Executes during object creation, after setting all properties.
function popupmenu_algorithms_CreateFcn(hObject, eventdata, handles)
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_START_Callback(hObject, eventdata, handles)
% Hints: get(hObject,'String') returns contents of edit_START as text
%        str2double(get(hObject,'String')) returns contents of edit_START as a double


% --- Executes during object creation, after setting all properties.
function edit_START_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_GOAL_Callback(hObject, eventdata, handles)
% Hints: get(hObject,'String') returns contents of edit_GOAL as text
%        str2double(get(hObject,'String')) returns contents of edit_GOAL as a double


% --- Executes during object creation, after setting all properties.
function edit_GOAL_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ob_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit_ob_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_DISPLAY_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_DISPLAY_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_START.
function pushbutton_START_Callback(hObject, eventdata, handles)
global P_start
P_start = [0 0];
plot(P_start(1,1),P_start(1,2),'ro')
hold on
set(handles.edit_START,'String',num2str(P_start));


% --- Executes on button press in pushbutton_GOAL.
function pushbutton_GOAL_Callback(hObject, eventdata, handles)
global P_goal
P_goal = [10 10];
plot(P_goal(1,1),P_goal(1,2),'ko')
set(handles.edit_GOAL,'String',num2str(P_goal));

% --- Executes on button press in pushbutton_CLC.
function pushbutton_CLC_Callback(hObject, eventdata, handles)
cla
