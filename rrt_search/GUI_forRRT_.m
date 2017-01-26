function varargout = GUI_forRRT_(varargin)
% GUI_FORRRT_ MATLAB code for GUI_forRRT_.fig
%      GUI_FORRRT_, by itself, creates a new GUI_FORRRT_ or raises the existing
%      singleton*.
%
%      H = GUI_FORRRT_ returns the handle to a new GUI_FORRRT_ or the handle to
%      the existing singleton*.
%
%      GUI_FORRRT_('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_FORRRT_.M with the given input arguments.
%
%      GUI_FORRRT_('Property','Value',...) creates a new GUI_FORRRT_ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_forRRT__OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_forRRT__OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_forRRT_

% Last Modified by GUIDE v2.5 26-Jan-2017 15:17:03

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
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_forRRT_ (see VARARGIN)

% Choose default command line output for GUI_forRRT_
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_forRRT_ wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_forRRT__OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_RUN_MAIN.
function pushbutton_RUN_MAIN_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_RUN_MAIN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_START_Callback(hObject, eventdata, handles)
% hObject    handle to edit_START (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_START as text
%        str2double(get(hObject,'String')) returns contents of edit_START as a double


% --- Executes during object creation, after setting all properties.
function edit_START_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_START (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_GOAL_Callback(hObject, eventdata, handles)
% hObject    handle to edit_GOAL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_GOAL as text
%        str2double(get(hObject,'String')) returns contents of edit_GOAL as a double


% --- Executes during object creation, after setting all properties.
function edit_GOAL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_GOAL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ob_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ob (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_ob as text
%        str2double(get(hObject,'String')) returns contents of edit_ob as a double


% --- Executes during object creation, after setting all properties.
function edit_ob_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ob (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
