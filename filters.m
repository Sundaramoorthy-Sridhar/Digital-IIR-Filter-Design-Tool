function varargout = filters(varargin)
% FILTERS MATLAB code for filters.fig
%      FILTERS, by itself, creates a new FILTERS or raises the existing
%      singleton*.
%
%      H = FILTERS returns the handle to a new FILTERS or the handle to
%      the existing singleton*.
%
%      FILTERS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FILTERS.M with the given input arguments.
%
%      FILTERS('Property','Value',...) creates a new FILTERS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before filters_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to filters_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help filters

% Last Modified by GUIDE v2.5 28-Sep-2019 20:24:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @filters_OpeningFcn, ...
                   'gui_OutputFcn',  @filters_OutputFcn, ...
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


% --- Executes just before filters is made visible.
function filters_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to filters (see VARARGIN)

% Choose default command line output for filters
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes filters wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = filters_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function editAp_Callback(hObject, eventdata, handles)
% hObject    handle to editAp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Ap=str2num(get(handles.editAp,'string'));
% Hints: get(hObject,'String') returns contents of editAp as text
%        str2double(get(hObject,'String')) returns contents of editAp as a double


% --- Executes during object creation, after setting all properties.
function editAp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAs_Callback(hObject, eventdata, handles)
% hObject    handle to editAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%As=str2num(get(handles.editAs,'string'));
% Hints: get(hObject,'String') returns contents of editAs as text
%        str2double(get(hObject,'String')) returns contents of editAs as a double


% --- Executes during object creation, after setting all properties.
function editAs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editWp_Callback(hObject, eventdata, handles)
% hObject    handle to editWp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%PEF_D=str2num(get(handles.editWp,'string'))*pi;
% Hints: get(hObject,'String') returns contents of editWp as text
%        str2double(get(hObject,'String')) returns contents of editWp as a double


% --- Executes during object creation, after setting all properties.
function editWp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editWp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editWs_Callback(hObject, eventdata, handles)
% hObject    handle to editWs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%SEF_D=str2num(get(handles.editWs,'string'))*pi;
% Hints: get(hObject,'String') returns contents of editWs as text
%        str2double(get(hObject,'String')) returns contents of editWs as a double


% --- Executes during object creation, after setting all properties.
function editWs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editWs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Ap=str2double(get(handles.editAp,'string'));
As=str2double(get(handles.editAs,'string'));
PEF_D=str2double(get(handles.editWp,'string'))*pi;
SEF_D=str2double(get(handles.editWs,'string'))*pi;
T=str2double(get(handles.T,'string'));
alpha_p=-20*log10(Ap);
alpha_s=-20*log10(As);
filter_choice=get(handles.filter_choice,'value');
ch=get(handles.choice,'value');
switch filter_choice
    case 1
      if ch==1 %low pass
      PEF_A=(2/T)*tan((PEF_D)/2);
      SEF_A=(2/T)*tan((SEF_D)/2);
      [N,CF]=buttord(PEF_A,SEF_A,alpha_p,alpha_s,'s'); %order and cutoff frequency
      [B,A]=butter(N,CF,'s');%unnormalized transfer function
      Hs=tf(B,A);
      H=evalc('Hs');
      set(handles.analog_tf,'string',H);
      [num,den]=bilinear(B,A,(1/T));
      Hz=tf(num,den,T);
      H1=evalc('Hz');
      set(handles.digital_tf,'string',H1);
      w=0:pi/16:pi;
      Hw=freqz(num,den,w);
      Hw_mag=abs(Hw);
      ylabel('Magnitude','fontweight','b');
      xlabel('Normalized frequency,\omega/\pi','fontweight','b');
      plot(w/pi,Hw_mag,'b');
      else %high pass
        PEF_A=(1/T)*PEF_D;
        SEF_A=(1/T)*SEF_D;
        [N,CF]=buttord(PEF_A,SEF_A,alpha_p,alpha_s,'s'); %order and cutoff frequency
        [B,A]=butter(N,CF,'s');%unnormalized transfer function
        Hs=tf(B,A);
        H=evalc('Hs');
        set(handles.analog_tf,'string',H);
        [num,den]=impinvar(B,A,(1/T));
        Hz=tf(num,den,T);
        H1=evalc('Hz');
        set(handles.digital_tf,'string',H1);
        w=0:pi/16:pi;
        Hw=freqz(num,den,w);
        Hw_mag=abs(Hw);
        ylabel('Magnitude','fontweight','b');
        xlabel('Normalized frequency,\omega/\pi','fontweight','b');
        plot(w/pi,Hw_mag,'b');
      end
    case 2
        if ch==1 %low pass
      PEF_A=(2/T)*tan((PEF_D)/2);
      SEF_A=(2/T)*tan((SEF_D)/2);
      [N,CF]=buttord(PEF_A,SEF_A,alpha_p,alpha_s,'s'); %order and cutoff frequency
      [B,A]=butter(N,CF,'high','s');%unnormalized transfer function
      Hs=tf(B,A);
      H=evalc('Hs');
      set(handles.analog_tf,'string',H);
      [num,den]=bilinear(B,A,(1/T));
      Hz=tf(num,den,T);
      H1=evalc('Hz');
      set(handles.digital_tf,'string',H1);
      w=0:pi/16:pi;
      Hw=freqz(num,den,w);
      Hw_mag=abs(Hw);
      plot(w/pi,Hw_mag,'b');
      ylabel('Magnitude','fontweight','b');
      xlabel('Normalized frequency,\omega/\pi','fontweight','b');
        else %high pass
       set(handles.digital_tf,'string','UNDEFINED');
       set(handles.analog_tf,'string','UNDEFINED');
       ylabel('Magnitude','fontweight','b');
       xlabel('Normalized frequency,\omega/\pi','fontweight','b');
       plot(0,0);
        end
end


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all;
close



function T_Callback(hObject, eventdata, handles)

function T_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function choice_Callback(hObject, eventdata, handles)

function choice_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function filter_choice_Callback(hObject, eventdata, handles)

function filter_choice_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over text3.
function text3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to text3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
