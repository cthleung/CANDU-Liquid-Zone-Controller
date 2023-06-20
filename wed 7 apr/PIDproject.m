%Written by Josh Henriques

function varargout = PIDproject(varargin)
% PIDPROJECT MATLAB code for PIDproject.fig
%      PIDPROJECT, by itself, creates a new PIDPROJECT or raises the existing
%      singleton*.
%
%      H = PIDPROJECT returns the handle to a new PIDPROJECT or the handle to
%      the existing singleton*.
%
%      PIDPROJECT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PIDPROJECT.M with the given input arguments.
%
%      PIDPROJECT('Property','Value',...) creates a new PIDPROJECT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PIDproject_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PIDproject_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PIDproject

% Last Modified by GUIDE v2.5 30-Mar-2021 15:55:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PIDproject_OpeningFcn, ...
                   'gui_OutputFcn',  @PIDproject_OutputFcn, ...
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

end

% --- Executes just before PIDproject is made visible.
function PIDproject_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PIDproject (see VARARGIN)
global comport
global RUN
global NPOINTS
if ~isempty(instrfind)
     fclose(instrfind);
      delete(instrfind);
end 
RUN = 0;
NPOINTS = 400;
comport = serial('COM10','BaudRate',115200);
fopen(comport)
% Choose default command line output for PIDproject
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PIDproject wait for user response (see UIRESUME)
% uiwait(handles.figure1);
 
end

% --- Outputs from this function are returned to the command line.
function varargout = PIDproject_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end 

% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global kp 
    global ki
    global kd
    global power
    global comport
    global RUN
    global NPOINTS
    
    if(isnan(kp) || isnan(ki) || isnan(kd) || isnan(power) || kp<0 || ki<0 || kd<0 || power<0)
        error("All values must be positive");
    elseif(power > 100)
         error("Power must be less than 100");
    end 
    
     fwrite(comport,power,'uint8')

     fwrite(comport,kp,'uint8')

     fwrite(comport,ki,'uint8')

     fwrite(comport,kd,'uint8')
    
    if RUN == 0
       RUN = 1;
        % change the string on the button to STOP
         set(handles.run,'string','STOP')
        while RUN == 1
            % send a single character prompt to the MCU
            fprintf(comport,'%s','U')
            % fetch data as single 8-bit bytes
            d = fread(comport,NPOINTS,'uint8');
            % plot the data on the screen
            plot(d*0.003)
            % Here are examples on how to set graph title, labels and axes
            title('EZ Scope');
            xlabel('Time - sec')
            ylabel('Volts')
            %axis ([0 400 0 240])
            xlim ([0 400]);
            ylim ([0 1]);
            % use drawnow to update the figure
            drawnow   
        end
    else
      RUN = 0;
      % change the string on the button to RUN
      set(handles.run,'string','RUN')
    end
    
end

% --- Executes on button press in quit.
function quit_Callback(hObject, eventdata, handles)
% hObject    handle to quit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global comport
global RUN
RUN = 0;
fclose(comport)
clear comport
% use close to terminate your program
% use quit to terminate MATLAB
close
end

function kp_Callback(hObject, eventdata, handles)
% hObject    handle to kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kp as text
%        str2double(get(hObject,'String')) returns contents of kp as a double
    global kp 
    kp = str2double(get(hObject,'String')); 
end

% --- Executes during object creation, after setting all properties.
function kp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

function ki_Callback(hObject, eventdata, handles)
% hObject    handle to ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ki as text
%        str2double(get(hObject,'String')) returns contents of ki as a double
    global ki 
    ki = str2double(get(hObject,'String')); 
end

% --- Executes during object creation, after setting all properties.
function ki_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

function kd_Callback(hObject, eventdata, handles)
% hObject    handle to kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kd as text
%        str2double(get(hObject,'String')) returns contents of kd as a double
    global kd 
    kd = str2double(get(hObject,'String')); 
end

% --- Executes during object creation, after setting all properties.
function kd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

function power_Callback(hObject, eventdata, handles)
% hObject    handle to power (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of power as text
%        str2double(get(hObject,'String')) returns contents of power as a double
    global power
    power = str2double(get(hObject,'String')); 
end

% --- Executes during object creation, after setting all properties.
function power_CreateFcn(hObject, eventdata, handles)
% hObject    handle to power (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

end


