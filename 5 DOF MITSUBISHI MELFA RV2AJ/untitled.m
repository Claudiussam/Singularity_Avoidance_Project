function varargout = app1(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 13-Jun-2021 19:07:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double


% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
syms th0 th1 th3 th4

Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;
Th_4 = str2double(handles.Theta_4.String)*pi/180;


L_1 = 0;
L_2 = 13;
L_3 = 4.5;
L_4 = 11.75;
L_5 = 6.5;
L_6 = 0;
%%
DH = [0 13 4.5 11.75 6.5 0;           % a
    2.2 0 0 0 0 3;                   % d
    pi/2 0 0 0 -pi/2 0;                % alpha
    th0' th1' pi/2 th3' th4' pi/2]'   % theta
%%
%      theta, d, a , alpha
L(1) = Link([0 2.2 L_1 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);
L(5) = Link([0 0 L_5 -pi/2]);
L(6) = Link([0 3 L_6 0]);
%%
angle_offset = [0 90 0 0]*pi/180; % (Motor angle) - (actual x-axis)
% angle_range_motor = [-100 100; -75 75; -215 5; -65 125]*pi/180;
%%
Robot = SerialLink(L); % Creating robot line model
Robot.name = 'DEU Robot';

try
theta_initial=evalin('base','theta_initial')+angle_offset;
catch
theta_initial=angle_offset;
end


Th_path{1}=linspace(theta_initial(1),Th_1,20);
Th_path{2}=linspace(theta_initial(2),Th_2+pi/2,20);
Th_path{3}=linspace(theta_initial(3),Th_3,20);
Th_path{4}=linspace(theta_initial(4),Th_4,20);

%% initial position
P_path=[];
for i=1:1:20
P_path=[P_path fkine4DOF(DH,[Th_path{1}(i) Th_path{2}(i) Th_path{3}(i) Th_path{4}(i)])];
end

%%
plot3(P_path(1,:),P_path(2,:),P_path(3,:),'Color',[1 0 0],'LineWidth',2);
hold on;
xlim([-30 30])
ylim([-30 30])
zlim([-10 30])

for i=1:20
Robot.plot([Th_path{1}(i) Th_path{2}(i) pi/2 Th_path{3}(i) Th_path{4}(i) pi/2],'scale',0.5,'perspective','jointdiam',2,'jaxes','shadow'); % hareketli
end

P=fkine4DOF(DH,[Th_1 Th_2 Th_3 Th_4],angle_offset);
P=eval(P)
handles.Pos_X.String = num2str(round(P(1),3));
handles.Pos_Y.String = num2str(round(P(2),3));
handles.Pos_Z.String = num2str(round(P(3),3));

assignin('base','theta_initial',[Th_1 Th_2 Th_3 Th_4]); 

function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
syms th0 th1 th3 th4
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);

angle_offset = [0 90 0 0]*pi/180; % (Motor angle) - (actual x-axis)
angle_range_motor = [-100 100; -75 75; -125 95; -155 35]*pi/180;
% %%
% Th_1 = str2double(handles.Theta_1.String)*pi/180;
% Th_2 = str2double(handles.Theta_2.String)*pi/180+pi/2;
% Th_3 = str2double(handles.Theta_3.String)*pi/180+pi/2;
% Th_4 = str2double(handles.Theta_4.String)*pi/180-pi/2;
%%
L_1 = 0;
L_2 = 13;
L_3 = 4.5;
L_4 = 11.75;
L_5 = 6.5;
L_6 = 0;
%%
DH = [0 13 4.5 11.75 6.5 0;           % a
    2.2 0 0 0 0 3;                   % d
    pi/2 0 0 0 -pi/2 0;                % alpha
    th0' th1' pi/2 th3' th4' pi/2]'   % theta
%%
%      theta, d, a , alpha
L(1) = Link([0 2.2 L_1 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);
L(5) = Link([0 0 L_5 -pi/2]);
L(6) = Link([0 3 L_6 0]);

Robot = SerialLink(L);
Robot.name = 'DEU Robot';

T = [ 1 0 0 PX;
      0 1 0 PY;
      0 0 1 PZ;
      0 0 0 1];
  
%%
n=10;
try
initial_position=evalin('base','initial_position');
catch
initial_position=fkine4DOF(DH,angle_offset);
end

assignin('base','initial_position',[PX PY PZ]); 

P_path_1=linspace(initial_position(1),PX,n);
P_path_2=linspace(initial_position(2),PY,n);
P_path_3=linspace(initial_position(3),PZ,n);
%%

%% initial position
T_path=[];
for i=1:1:n
T_path=[T_path ikine4DOF_v2(DH,[P_path_1(i) P_path_2(i) P_path_3(i)],angle_range_motor,angle_offset)];
end
%% 
plot3(P_path_1,P_path_2,P_path_3,'Color',[1 0 0],'LineWidth',2);
hold on;
xlim([-30 30])
ylim([-30 30])
zlim([-10 30])

for i=1:n
Robot.plot([T_path(1,i) T_path(2,i)+pi/2 pi/2 T_path(4,i) T_path(5,i) pi/2],'scale',0.5,'perspective','jointdiam',2,'jaxes','shadow'); % hareketli
end
J=ikine4DOF_v2(DH,[PX PY PZ],angle_range_motor,angle_offset);
J_deg = J*180/pi % theta
handles.Theta_1.String = num2str(round(J_deg(1),3));
handles.Theta_2.String = num2str(round(J_deg(2),3));
handles.Theta_3.String = num2str(round(J_deg(4),3));
handles.Theta_4.String = num2str(round(J_deg(5),3));
