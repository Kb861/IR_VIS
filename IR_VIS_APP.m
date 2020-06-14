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

% Last Modified by GUIDE v2.5 14-Jun-2020 12:41:11

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

    indir=uigetdir;
    handles.uploadButtonIR =char(indir);
     
    ListOfImageNames = {};
    global folderIR
    folderIR = handles.uploadButtonIR;
    if length(folderIR) > 0 
        if exist(folderIR,'dir') == false
            msgboxw(['Folder ' folderIR ' does not exist.']);
            return;
        end

    else
        fprintf('No folder specified as input for function LoadImageList.\n');
        WarnUser('No folder specified as input for function LoadImageList.');
        return;
    end
    ImageFiles = dir([folderIR '\*.*']);
    for Index = 1:length(ImageFiles)
        baseFileName = ImageFiles(Index).name;
        [folder2, name, extension] = fileparts(baseFileName);
        extension = upper(extension);
        switch lower(extension)
       case {'.png'}          
            ListOfImageNames = [ListOfImageNames baseFileName];
        otherwise
        end
    end
   set(handles.listboxIR,'string',ListOfImageNames,'Value',1);
 

% --- Executes on button press in uploadButtonVIS.
function uploadButtonVIS_Callback(hObject, eventdata, handles)
% hObject    handle to uploadButtonVIS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    indir=uigetdir;
    handles.uploadButtonVIS =char(indir);
     
    ListOfImageNames = {};
    global folderVIS
    folderVIS = handles.uploadButtonVIS;
    if length(folderVIS) > 0 
        if exist(folderVIS,'dir') == false
            msgboxw(['Folder ' folderVIS ' does not exist.']);
            return;
        end

    else
        fprintf('No folder specified as input for function LoadImageList.\n');
        WarnUser('No folder specified as input for function LoadImageList.');
        return;
    end
    ImageFiles = dir([folderVIS '\*.*']);
    for Index = 1:length(ImageFiles)
        baseFileName = ImageFiles(Index).name;
        [folder2, name, extension] = fileparts(baseFileName);
        extension = upper(extension);
        switch lower(extension)
       case {'.jpg'}          
            ListOfImageNames = [ListOfImageNames baseFileName];
        otherwise
        end
    end
   set(handles.listboxVIS,'string',ListOfImageNames,'Value',1);


% --- Executes on selection change in listboxIR.
function listboxIR_Callback(hObject, eventdata, handles)
% hObject    handle to listboxIR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listboxIR contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listboxIR
global folderIR
selected_image_idx = get(handles.listboxIR, 'Value');
all_image_names = cellstr( get(handles.listboxIR, 'String') );
selected_image_names = all_image_names(selected_image_idx);
num_selected = length(selected_image_names);
for K = 1 : num_selected
  this_name = selected_image_names{K};
  this_image_file = fullfile(folderIR, this_name ); 
  this_image = imread(this_image_file);
  axes(handles.IRAxes);
  imshow(this_image);
end

% --- Executes during object creation, after setting all properties.
function listboxIR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listboxIR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listboxVIS.
function listboxVIS_Callback(hObject, eventdata, handles)
% hObject    handle to listboxVIS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listboxVIS contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listboxVIS
global folderVIS
selected_image_idx = get(handles.listboxVIS, 'Value');
all_image_names = cellstr( get(handles.listboxVIS, 'String') );
selected_image_names = all_image_names(selected_image_idx);
num_selected = length(selected_image_names);
for K = 1 : num_selected
  this_name = selected_image_names{K};
  this_image_file = fullfile(folderVIS, this_name );
  this_image = imread(this_image_file);
  axes(handles.VISAxes);
  imshow(this_image);
end

% --- Executes during object creation, after setting all properties.
function listboxVIS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listboxVIS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function IRAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IRAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'xtick',[])
set(gca,'ytick',[])
% Hint: place code in OpeningFcn to populate IRAxes


% --- Executes during object creation, after setting all properties.
function VISAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VISAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'xtick',[])
set(gca,'ytick',[])
% Hint: place code in OpeningFcn to populate VISAxes


% --- Executes on button press in markersBtn1.
function markersBtn1_Callback(hObject, eventdata, handles)
% hObject    handle to markersBtn1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in markersBtn2.
function markersBtn2_Callback(hObject, eventdata, handles)
% hObject    handle to markersBtn2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in calibrateBtn.
function calibrateBtn_Callback(hObject, eventdata, handles)
% hObject    handle to calibrateBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
