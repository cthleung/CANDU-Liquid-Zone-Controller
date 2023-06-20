%-----------------------------------------------------------------------
% myscope.m
% Written by Kenrick Chin
% Date: 2016 Jan 29
%-----------------------------------------------------------------------
function varargout = myscope(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myscope_OpeningFcn, ...
                   'gui_OutputFcn',  @myscope_OutputFcn, ...
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
%-----------------------------------------------------------------------
% --- Executes just before myscope is made visible.
function myscope_OpeningFcn(hObject, eventdata, handles, varargin)
global comport
global RUN
global NPOINTS
RUN = 0;
NPOINTS = 400;
comport = serial('COM4','BaudRate',115200);
fopen(comport)
% Choose default command line output for myscope
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
%-----------------------------------------------------------------------
function myscope_OutputFcn(hObject, eventdata, handles, varargin)
%-----------------------------------------------------------------------
function Quit_Button_Callback(hObject, eventdata, handles)
global comport
global RUN
RUN = 0;
fclose(comport)
clear comport
% use close to terminate your program
% use quit to terminate MATLAB
close
%----------------------------------------------------------------------
function Run_Button_Callback(hObject, eventdata, handles)
global comport
global NPOINTS
global RUN
if RUN == 0
  RUN = 1;
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  while RUN == 1
    % send a single character prompt to the MCU
    fprintf(comport,'%s','U')
    % fetch data as single 8-bit bytes
    d = fread(comport,NPOINTS,'uint8');
    t = linspace(0,20e-4,400);
    %t=0:0.01:1.736e-5;
    % plot the data on the screen
    plot(t,d*0.003)
    % Here are examples on how to set graph title, labels and axes
   
    title('EZ Scope');
    xlabel('Time - sec')
    ylabel('Volts')
    xlim([0 10e-4]);
    ylim([0 0.5]);
    %axis ([0 1.875e-7 0 0.6])
    % use drawnow to update the figure
    drawnow  
  end
else
  RUN = 0;
  % change the string on the button to RUN
  set(handles.Run_Button,'string','RUN')
end
%----------------------------------------------------------------------