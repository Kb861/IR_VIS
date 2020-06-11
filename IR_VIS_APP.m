function varargout = IR_VIS_APP(varargin)
% IR_VIS_APP MATLAB code for IR_VIS_APP.fig
%      IR_VIS_APP, by itself, creates a new IR_VIS_APP or raises the existing
%      singleton*.
%
%      H = IR_VIS_APP returns the handle to a new IR_VIS_APP or the handle to
%      the existing singleton*.
%
%      IR_VIS_APP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IR_VIS_APP.M with the given input arguments.
%
%      IR_VIS_APP('Property','Value',...) creates a new IR_VIS_APP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IR_VIS_APP_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IR_VIS_APP_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IR_VIS_APP

% Last Modified by GUIDE v2.5 11-Jun-2020 20:51:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IR_VIS_APP_OpeningFcn, ...
                   'gui_OutputFcn',  @IR_VIS_APP_OutputFcn, ...
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


% --- Executes just before IR_VIS_APP is made visible.
function IR_VIS_APP_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IR_VIS_APP (see VARARGIN)

% Choose default command line output for IR_VIS_APP
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IR_VIS_APP wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = IR_VIS_APP_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in uploadButtonIR.
function uploadButtonIR_Callback(hObject, eventdata, handles)
% hObject    handle to uploadButtonIR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    [fileName, pathName] = uigetfile('*.PNG', 'All Files (*.*)');
    global image
    image.infoIR = [fileName, pathName];
    image.imreadIR = imread([pathName, fileName]);
end

% --- Executes on button press in uploadButtonVIS.
function uploadButtonVIS_Callback(hObject, eventdata, handles)
% hObject    handle to uploadButtonVIS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    [fileName, pathName] = uigetfile('*.JPG', 'All Files (*.*)');
    global image
    image.infoVIS = [fileName, pathName];
    image.imreadVIS = imread([pathName, fileName]);

end