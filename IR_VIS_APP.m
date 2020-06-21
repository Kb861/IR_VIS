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

% Last Modified by GUIDE v2.5 17-Jun-2020 20:56:17

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
    ImageFiles = dir([folderIR '/*.*']);
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
    ImageFiles = dir([folderVIS '/*.*']);
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
global this_image_IR
selected_image_idx = get(handles.listboxIR, 'Value');
all_image_names = cellstr( get(handles.listboxIR, 'String') );
selected_image_names = all_image_names(selected_image_idx);
num_selected = length(selected_image_names);
for K = 1 : num_selected
  this_name = selected_image_names{K};
  this_image_file = fullfile(folderIR, this_name ); 
  this_image_IR = imread(this_image_file);
  axes(handles.IRAxes);
  imshow(this_image_IR);
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
global this_image_VIS
selected_image_idx = get(handles.listboxVIS, 'Value');
all_image_names = cellstr( get(handles.listboxVIS, 'String') );
selected_image_names = all_image_names(selected_image_idx);
num_selected = length(selected_image_names);
for K = 1 : num_selected
  this_name = selected_image_names{K};
  this_image_file = fullfile(folderVIS, this_name );
  this_image_VIS = imread(this_image_file);
  axes(handles.VISAxes);
  imshow(this_image_VIS);
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
global xIR;
global yIR;

n=1;
while n < 5
[xIR(n),yIR(n)] = ginput(1);
hold(handles.IRAxes,'on'); 
drawnow;
plot(xIR(n), yIR(n), 'yo', 'MarkerSize', 10);
n=n+1;
end

% --- Executes on button press in markersBtn2.
function markersBtn2_Callback(hObject, eventdata, handles)
% hObject    handle to markersBtn2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n=1;
global xVIS;
global yVIS;

while n < 5
[xVIS(n),yVIS(n)] = ginput(1);
hold(handles.VISAxes,'on'); 
drawnow;
plot(xVIS(n), yVIS(n), 'yo', 'MarkerSize', 10);
n=n+1;
end

% --- Executes on button press in calibrateBtn.
function calibrateBtn_Callback(hObject, eventdata, handles)
% hObject    handle to calibrateBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xIR;
global yIR;
global xVIS;
global yVIS;
global folderVIS
global folderIR
global data2
listBoxStrings = cellstr(get(handles.listboxVIS,'String'));
listBoxStrings2 = cellstr(get(handles.listboxIR,'String'));
 
 


num_selected = length(listBoxStrings);
for K = 1 : num_selected
  this_name = listBoxStrings{K};
  this_image_file = fullfile(folderVIS, this_name ); 
   this_image_VIS = imread(this_image_file);

    this_name = listBoxStrings2{K};
  this_image_file = fullfile(folderIR, this_name ); 
   this_image_IR = imread(this_image_file);

[y, x, z] = size(this_image_VIS);
picture_Vis = uint8(zeros(y*2, x*2, z));
picture_Vis(floor(y/2):y+floor(y/2)-1, floor(x/2):x+floor(x/2)-1, :) = this_image_VIS;
picture_Vis_X = xVIS + floor(x/2)-1;
picture_Vis_Y = yVIS + floor(y/2)-1;
    
VisMinX = find(picture_Vis_X == min(picture_Vis_X));
VisMaxX = find(picture_Vis_X == max(picture_Vis_X));
VisMinY = find(picture_Vis_Y == min(picture_Vis_Y));
VisMaxY = find(picture_Vis_Y == max(picture_Vis_Y));
    
IrMinX = find(xIR == min(xIR));
IrMaxX = find(xIR == max(xIR));
IrMinY = find(yIR == min(yIR));
IrMaxY = find(yIR == max(yIR));   
IrX = picture_Vis_X(VisMinX) - xIR(IrMinX);
IrXResult = xIR + IrX;
    
VisXDist = picture_Vis_X(VisMaxX) - picture_Vis_X(VisMinX);
IrXDist = xIR(IrMaxX) - xIR(IrMinX);
AsX = mean(IrXDist) / mean(VisXDist);  
[y,x,z] = size(picture_Vis);
VisCropXres = picture_Vis_X * AsX;        
IrY = mean(picture_Vis_Y(VisMinY)) - mean(yIR(IrMinY));
IrYResult = yIR + IrY;
VisYDist = picture_Vis_Y(VisMaxY) - picture_Vis_Y(VisMinY);
IrYDist = yIR(IrMaxY) - yIR(IrMinY);
AsY = mean(IrYDist) / mean(VisYDist);
VisCropYres = picture_Vis_Y * AsY;
    
mergerdIMG = [];
mergerdIMG = imresize(picture_Vis, [y*AsY, x*AsX]);
    
IrMeanX = mean(xIR);
IrMeanY = mean(yIR);
VisMeanX = mean(VisCropXres);
VisMeanY = mean(VisCropYres);  
difX = abs(VisMeanX) - abs(IrMeanX);
difY = abs(VisMeanY) - abs(IrMeanY);
[y,x,z] = size(this_image_IR);  
[y1, x1, z1] = size(mergerdIMG);
    
maxX = x+difX-1;
maxY = y+difY-1;
if maxY > y1
	yDiff = maxY - y;
	maxY = y1 - 1;
end
if maxX > x1
	xDiff = maxX - x;
	maxX = x1 - 1;
end
    
croppedVis = mergerdIMG(difY:maxY, difX:maxX, :);       
[y,x,z] = size(this_image_IR);
maskProg = ones(y,x);
level = graythresh(this_image_IR(:,:,1));
maskProg = im2bw(this_image_IR(:,:,1), level);
maskProg3 = repmat(maskProg, [1, 1, 3]);
imCropMasked = croppedVis;
imCropMasked(~maskProg3) = 0;
VisPlusIr = imCropMasked;
VisPlusIr(:,:,1) =  VisPlusIr(:,:,1)/2 + this_image_IR(:,:,1)/2 ;
VisPlusIr(:,:,2) = VisPlusIr(:,:,2)/2 + this_image_IR(:,:,2)/2 ;
VisPlusIr(:,:,3) = VisPlusIr(:,:,3)/2 + this_image_IR(:,:,3)/2 ;
data2(K).data=VisPlusIr;
end
axes(handles.IRVISAxes);
imshow(data2(1).data);


% --- Executes on button press in stat1Btn.
function stat1Btn_Callback(hObject, eventdata, handles)
% hObject    handle to stat1Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
IM1=getimage(handles.IRAxes);
IM2=getimage(handles.IRVISAxes);
IM3=getimage(handles.TransforAxes);
R1=get(handles.histBtn,'Value')
if(R1 == 1)
Red = IM1(:,:,1);
axes(handles.origAxes);
imhist(Red);

Red2 = IM2(:,:,1);
axes(handles.trans1Axes);
imhist(Red2);

Red3 = IM3(:,:,1);
axes(handles.trans2Axes);
imhist(Red3);

end
R2=get(handles.minMaxBtn,'Value')
if(R2 == 1)
Blue1 = IM1(:,:,3);
axes(handles.origAxes);
imhist(Blue1);

Blue2 = IM2(:,:,3);
axes(handles.trans1Axes);
imhist(Blue2);

Blue3 = IM3(:,:,3);
axes(handles.trans2Axes); 
imhist(Blue3);

end
R3=get(handles.greenBtn,'Value')
if(R3 == 1)
Green1 = IM1(:,:,2);
axes(handles.origAxes);
imhist(Green1);

Green2 = IM2(:,:,2);
axes(handles.trans1Axes);
imhist(Green2);

Green3 = IM3(:,:,2);
axes(handles.trans2Axes); 
imhist(Green3);

end
guidata(hObject, handles);
% --- Executes on button press in stat2Btn.
function stat2Btn_Callback(hObject, eventdata, handles)
% hObject    handle to stat2Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global data3;
global folderIR
global data2
n=1;
while n < 2
[xIR(n),yIR(n)] = ginput(1);
drawnow;
n=n+1;
end

listBoxStrings2 = cellstr(get(handles.listboxIR,'String'));
 
this_name = listBoxStrings2{1};
this_image_file = fullfile(folderIR, this_name ); 
this_image_IR = imread(this_image_file);

temp1 = rgb2gray(this_image_IR);
temp2 = rgb2gray(data2(1).data);
temp3 = rgb2gray(data3(1).data);

tempVal1=double(2/(double(max(temp1(1,:))-min(temp1(1,:)))));
tempVal2=double(2/(double(max(temp2(1,:))-min(temp2(1,:)))));
tempVal3=double(2/(double(max(temp3(1,:))-min(temp3(1,:)))));
t1=[];
t2=[];
t3=[];
startTemp=28;
for i=2:30
this_name = listBoxStrings2{i};
this_image_file = fullfile(folderIR, this_name ); 
this_image_IR = imread(this_image_file);
I1 = imcrop(rgb2gray(this_image_IR),[xIR-2 yIR-2 5 5]);
meanI1 = mean(mean(I1));
value1=startTemp + (meanI1*tempVal1);
t1=[t1 value1];
I2 = imcrop(rgb2gray(data2(i).data),[xIR-2 yIR-2 5 5]);
meanI2 = mean(mean(I2));
value2=startTemp + (meanI2*tempVal2);
t2=[t2 value2];
I3 = imcrop(rgb2gray(data2(i).data),[xIR-2 yIR-2 5 5]);
meanI3 = mean(mean(I3));
value3=startTemp + (meanI3*tempVal3);
t3=[t3 value3];
end
time = 1:29;
axes(handles.origAxes);
plot(time,t1)

axes(handles.trans1Axes);
plot(time,t2)

axes(handles.trans2Axes); 
plot(time,t3)

% --- Executes on button press in trans2Btn.
function trans2Btn_Callback(hObject, eventdata, handles)
% hObject    handle to trans2Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global folderVIS
global xIR;
global yIR;
global xVIS;
global yVIS;
global data2;
global data3;
listBoxStrings = cellstr(get(handles.listboxVIS,'String'));

this_name = listBoxStrings{1};
this_image_file = fullfile(folderVIS, this_name ); 
this_image2 = imread(this_image_file);
[y,x,z] = size(this_image2);
xProc = x*0.05;
yProc = y*0.05;
minX=min(xVIS);
minY=min(yVIS);
maxX=min(xVIS);
maxY=min(xVIS);
data1(1).data= this_image2;
num_selected = length(listBoxStrings);
I2 = imcrop(this_image2,[minX-xProc minY-yProc maxX+xProc maxY+yProc]);
data1(1).data= I2;
for K = 2 : num_selected
  this_name = listBoxStrings{K};
  this_image_file = fullfile(folderVIS, this_name ); 
  this_image = imread(this_image_file);
  I3 = imcrop(this_image,[minX-xProc minY-yProc maxX+xProc maxY+yProc]);
data1(K).data = imabsdiff(I3,I2);
end
for K = 1 : num_selected
this_image_VIS=data1(K).data;
this_image_IR=data2(K).data;
[y, x, z] = size(this_image_VIS);
picture_Vis = uint8(zeros(y*2, x*2, z));
picture_Vis(floor(y/2):y+floor(y/2)-1, floor(x/2):x+floor(x/2)-1, :) = this_image_VIS;
picture_Vis_X = xVIS + floor(x/2)-1;
picture_Vis_Y = yVIS + floor(y/2)-1;
    
VisMinX = find(picture_Vis_X == min(picture_Vis_X));
VisMaxX = find(picture_Vis_X == max(picture_Vis_X));
VisMinY = find(picture_Vis_Y == min(picture_Vis_Y));
VisMaxY = find(picture_Vis_Y == max(picture_Vis_Y));
    
IrMinX = find(xIR == min(xIR));
IrMaxX = find(xIR == max(xIR));
IrMinY = find(yIR == min(yIR));
IrMaxY = find(yIR == max(yIR));   
IrX = picture_Vis_X(VisMinX) - xIR(IrMinX);
IrXResult = xIR + IrX;
    
VisXDist = picture_Vis_X(VisMaxX) - picture_Vis_X(VisMinX);
IrXDist = xIR(IrMaxX) - xIR(IrMinX);
AsX = mean(IrXDist) / mean(VisXDist);  
[y,x,z] = size(picture_Vis);
VisCropXres = picture_Vis_X * AsX;        
IrY = mean(picture_Vis_Y(VisMinY)) - mean(yIR(IrMinY));
IrYResult = yIR + IrY;
VisYDist = picture_Vis_Y(VisMaxY) - picture_Vis_Y(VisMinY);
IrYDist = yIR(IrMaxY) - yIR(IrMinY);
AsY = mean(IrYDist) / mean(VisYDist);
VisCropYres = picture_Vis_Y * AsY;
    
mergerdIMG = [];
mergerdIMG = imresize(picture_Vis, [y*AsY, x*AsX]);
    
IrMeanX = mean(xIR);
IrMeanY = mean(yIR);
VisMeanX = mean(VisCropXres);
VisMeanY = mean(VisCropYres);  
difX = abs(VisMeanX) - abs(IrMeanX);
difY = abs(VisMeanY) - abs(IrMeanY);
[y,x,z] = size(this_image_IR);  
[y1, x1, z1] = size(mergerdIMG);
    
maxX = x+difX-1;
maxY = y+difY-1;
if maxY > y1
	yDiff = maxY - y;
	maxY = y1 - 1;
end
if maxX > x1
	xDiff = maxX - x;
	maxX = x1 - 1;
end
    
croppedVis = mergerdIMG(difY:maxY, difX:maxX, :); 
[s1,s2,s3]=size(croppedVis)
if(s2~= x) 
 croppedVis = mergerdIMG(difY:maxY, maxX-x:maxX-1, :);    
end
[y,x,z] = size(this_image_IR);
maskProg = ones(y,x);
level = graythresh(this_image_IR(:,:,1));
maskProg = im2bw(this_image_IR(:,:,1), level);
maskProg3 = repmat(maskProg, [1, 1, 3]);
imCropMasked = croppedVis;
imCropMasked(~maskProg3) = 0;
VisPlusIr = imCropMasked;
VisPlusIr(:,:,1) =  VisPlusIr(:,:,1)/2 + this_image_IR(:,:,1)/2 ;
VisPlusIr(:,:,2) = VisPlusIr(:,:,2)/2 + this_image_IR(:,:,2)/2 ;
VisPlusIr(:,:,3) = VisPlusIr(:,:,3)/2 + this_image_IR(:,:,3)/2 ;
data3(K).data=VisPlusIr;
end
axes(handles.TransforAxes);
imshow(data3(1).data);


% --- Executes on button press in next1Btn.
function next1Btn_Callback(hObject, eventdata, handles)
% hObject    handle to next1Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in next2Btn.
function next2Btn_Callback(hObject, eventdata, handles)
% hObject    handle to next2Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in prev1Btn.
function prev1Btn_Callback(hObject, eventdata, handles)
% hObject    handle to prev1Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in prev2Btn.
function prev2Btn_Callback(hObject, eventdata, handles)
% hObject    handle to prev2Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global data2
x = int32(get(hObject, 'Value'));
axes(handles.IRVISAxes);
imshow(data2(x).data);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global data3
x = int32(get(hObject, 'Value'));
axes(handles.TransforAxes);
imshow(data3(x).data);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
