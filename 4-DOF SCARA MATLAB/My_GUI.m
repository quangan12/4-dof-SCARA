function varargout = My_GUI(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @My_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @My_GUI_OutputFcn, ...
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


% --- Executes just before My_GUI is made visible.
function My_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
global myScara;
global the1_pre;
global the2_pre;
global d3_pre;
global the4_pre;

myScara = SCARA(handles,0,90,0,0); %truyen 4 thong so ban dau the1, the2, d3, the4 = 0 

handles.output = hObject;
guidata(hObject, handles);
VeRobot(myScara,handles,20,30); 
the1_pre = 0;
the2_pre = 90;
d3_pre = 0;
the4_pre = 0;
Trajectory_Plot(handles);

% UIWAIT makes My_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = My_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
theta_1 = get(handles.slider1,'value');
set(handles.edit1, 'string', num2str(theta_1));

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
theta_2 = get(handles.slider2,'value');
set(handles.edit2, 'string', num2str(theta_2));

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
d_3 = get(handles.slider3,'value');
set(handles.edit3, 'string', num2str(d_3));

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
theta_4 = get(handles.slider4,'value');
set(handles.edit4, 'string', num2str(theta_4));

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_forward.
function btn_forward_Callback(hObject, eventdata, handles)

global myScara
global the1_pre;
global the2_pre;
global d3_pre;
global the4_pre;

Trajectory_Plot(handles)

xxx = [];
yyy = [];
zzz = [];

%lay gia tri tu slider
the1 = get(handles.slider1,'value');
the2 = get(handles.slider2,'value');
d3 = get(handles.slider3,'value');
the4 = get(handles.slider4,'value');
v_max = str2double(get(handles.ip_vmax,'String'));
a_max = str2double(get(handles.ip_amax,'String'));

if(handles.lspb.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Trapezoidal_Vel_Profile(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Trapezoidal_Vel_Profile(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Trapezoidal_Vel_Profile(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Trapezoidal_Vel_Profile(abs(the4-the4_pre),v_max,a_max);
elseif(handles.scurve.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Scurve5Segment(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Scurve5Segment(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Scurve5Segment(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Scurve5Segment(abs(the4-the4_pre),v_max,a_max);    
end

%ve mo hinh tu gia tri lay duoc
for i = 1:1:length(t1)
    myScara = SCARA(handles,the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1(i),the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2(i),d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3(i),the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4(i));
    VeRobot(myScara,handles,20,30);
    xxx(i) = myScara.pos(4,1);
    yyy(i) = myScara.pos(4,2);
    zzz(i) = myScara.pos(4,3);
    
    plot(handles.t1, t1(1:i), the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1(1,1:i), 'b-');
    plot(handles.t1_dot, t1(1:i), ((the1-the1_pre)/abs(the1-the1_pre))*tt1_dot(1,1:i), 'b-');
    plot(handles.t1_2dot, t1(1:i), ((the1-the1_pre)/abs(the1-the1_pre))*tt1_2dot(1,1:i), 'b-');
    
    plot(handles.t2, t2(1:i), the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2(1,1:i), 'b-');
    plot(handles.t2_dot, t2(1:i), ((the2-the2_pre)/abs(the2-the2_pre))*tt2_dot(1,1:i), 'b-');
    plot(handles.t2_2dot, t2(1:i), ((the2-the2_pre)/abs(the2-the2_pre))*tt2_2dot(1,1:i), 'b-');
    
    plot(handles.d3, t3(1:i), d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3(1,1:i), 'b-');
    plot(handles.d3_dot, t3(1:i), ((d3-d3_pre)/abs(d3-d3_pre))*d3_dot(1,1:i), 'b-');
    plot(handles.d3_2dot, t3(1:i), ((d3-d3_pre)/abs(d3-d3_pre))*d3_2dot(1,1:i), 'b-');
    
    plot(handles.t4, t4(1:i), the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4(1,1:i), 'b-');
    plot(handles.t4_dot, t4(1:i), ((the4-the4_pre)/abs(the4-the4_pre))*tt4_dot(1,1:i), 'b-');
    plot(handles.t4_2dot, t4(1:i), ((the4-the4_pre)/abs(the4-the4_pre))*tt4_2dot(1,1:i), 'b-');
    
    pause(0.01);
end
plot3(handles.robot_plot, xxx, yyy, zzz,'MarkerSize', 5, 'Color', 'red');

the1_pre = the1;
the2_pre = the2;
d3_pre = d3;
the4_pre = the4;


%x,y,z,roll,pitch,yaw
set(handles.x,'String',num2str(myScara.pos(4,1)));
set(handles.y,'String',num2str(myScara.pos(4,2)));
set(handles.z,'String',num2str(myScara.pos(4,3)));
set(handles.roll,'String',num2str(myScara.orien(4,1)*180/pi));
set(handles.pitch,'String',num2str(myScara.orien(4,2)*180/pi));
set(handles.yaw,'String',num2str(myScara.orien(4,3)*180/pi));




function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch as text
%        str2double(get(hObject,'String')) returns contents of pitch as a double


% --- Executes during object creation, after setting all properties.
function pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaww (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaww as text
%        str2double(get(hObject,'String')) returns contents of yaww as a double


% --- Executes during object creation, after setting all properties.
function yaww_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
global myScara
global the1_pre;
global the2_pre;
global d3_pre;
global the4_pre;

Trajectory_Plot(handles)

x = str2double(get(handles.x,'String'));
y = str2double(get(handles.y,'String'));
z = str2double(get(handles.z,'String'));
yaw = str2double(get(handles.yaw,'String'));
xx = [];
yy = [];
zz = [];
[the1,the2,d3,the4] = myScara.InverseKinematic(myScara, x, y, z, yaw);
the1 = the1*180/pi;
the2 = the2*180/pi;
set(handles.edit1,'String',num2str(the1));
set(handles.edit2,'String',num2str(the2));
set(handles.edit3,'String',num2str(d3));
set(handles.edit4,'String',num2str(the4));

v_max = str2double(get(handles.ip_vmax,'String'));
a_max = str2double(get(handles.ip_amax,'String'));

if(handles.lspb.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Trapezoidal_Vel_Profile(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Trapezoidal_Vel_Profile(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Trapezoidal_Vel_Profile(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Trapezoidal_Vel_Profile(abs(the4-the4_pre),v_max,a_max);
elseif(handles.scurve.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Scurve5Segment(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Scurve5Segment(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Scurve5Segment(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Scurve5Segment(abs(the4-the4_pre),v_max,a_max);    
end

for i = 1:1:length(t1)
    myScara = SCARA(handles,the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1(i),the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2(i),d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3(i),the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4(i));
    VeRobot(myScara,handles,20,30);
    xx(i) = myScara.pos(4,1);
    yy(i) = myScara.pos(4,2);
    zz(i) = myScara.pos(4,3);
    plot(handles.t1, t1(1:i), the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1(1,1:i), 'b-');
    plot(handles.t1_dot, t1(1:i), ((the1-the1_pre)/abs(the1-the1_pre))*tt1_dot(1,1:i), 'b-');
    plot(handles.t1_2dot, t1(1:i), ((the1-the1_pre)/abs(the1-the1_pre))*tt1_2dot(1,1:i), 'b-');
    
    plot(handles.t2, t2(1:i), the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2(1,1:i), 'b-');
    plot(handles.t2_dot, t2(1:i), ((the2-the2_pre)/abs(the2-the2_pre))*tt2_dot(1,1:i), 'b-');
    plot(handles.t2_2dot, t2(1:i), ((the2-the2_pre)/abs(the2-the2_pre))*tt2_2dot(1,1:i), 'b-');
    
    plot(handles.d3, t3(1:i), d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3(1,1:i), 'b-');
    plot(handles.d3_dot, t3(1:i), ((d3-d3_pre)/abs(d3-d3_pre))*d3_dot(1,1:i), 'b-');
    plot(handles.d3_2dot, t3(1:i), ((d3-d3_pre)/abs(d3-d3_pre))*d3_2dot(1,1:i), 'b-');
    
    plot(handles.t4, t4(1:i), the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4(1,1:i), 'b-');
    plot(handles.t4_dot, t4(1:i), ((the4-the4_pre)/abs(the4-the4_pre))*tt4_dot(1,1:i), 'b-');
    plot(handles.t4_2dot, t4(1:i), ((the4-the4_pre)/abs(the4-the4_pre))*tt4_2dot(1,1:i), 'b-');
    
    pause(0.01);
end
the1_pre = the1;
the2_pre = the2;
d3_pre = d3;
the4_pre = the4;

plot3(handles.robot_plot, xx, yy, zz,'MarkerSize', 5, 'Color', 'red');


% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Coordinate.
function Coordinate_Callback(hObject, eventdata, handles)
global myScara
if get(hObject, 'Value')
    axes(handles.robot_plot);
    % plot coordinate
    A0_1 = Matran_A(myScara.a(1),myScara.alpha(1)*pi/180,myScara.d(1),myScara.theta(1)*pi/180) ;
    A1_2 = Matran_A(myScara.a(2),myScara.alpha(2)*pi/180,myScara.d(2),myScara.theta(2)*pi/180) ;
    A2_3 = Matran_A(myScara.a(3),myScara.alpha(3)*pi/180,myScara.d(3),myScara.theta(3)*pi/180) ;
    A3_4 = Matran_A(myScara.a(4),myScara.alpha(4)*pi/180,myScara.d(4),myScara.theta(4)*pi/180) ;
    A0_0=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    A0_2=A0_1*A1_2;
    A0_3=A0_1*A1_2*A2_3;
    A0_4=A0_1*A1_2*A2_3*A3_4;   % Te

    plot_coordinate(0,0,0.2,A0_0,'0');
    plot_coordinate(myScara.pos(1,1),myScara.pos(1,2),myScara.pos(1,3)+0.05,A0_1,'1');
    plot_coordinate(myScara.pos(2,1),myScara.pos(2,2),myScara.pos(2,3)+0.1,A0_2,'2');
    plot_coordinate(myScara.pos(3,1),myScara.pos(3,2),myScara.pos(3,3)+0.2,A0_3,'3');
    plot_coordinate(myScara.pos(4,1),myScara.pos(4,2),myScara.pos(4,3),A0_4,'4');
else
    VeRobot(myScara,handles,20,30); 
end


% --- Executes on button press in Workspace.
function Workspace_Callback(hObject, eventdata, handles)
global myScara
if get(hObject, 'Value')
    PlotWorkspace(myScara,handles);
else
    VeRobot(myScara,handles,20,30);  
end


% --- Executes on selection change in sel.
function sel_Callback(hObject, eventdata, handles)
% hObject    handle to sel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns sel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from sel
contents = cellstr(get(handles.sel, 'String'));
space_plot_type = contents{get(handles.sel, 'Value')};
if strcmp(space_plot_type, 'Joint space')
    set(handles.joint_sl,'Visible','on');
    set(handles.pid_sl,'Visible','off'); 
elseif  strcmp(space_plot_type, 'PID') 
    set(handles.joint_sl,'Visible','off');
    set(handles.pid_sl,'Visible','on');
end    


% --- Executes during object creation, after setting all properties.
function sel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ip_vmax_Callback(hObject, eventdata, handles)
% hObject    handle to ip_vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ip_vmax as text
%        str2double(get(hObject,'String')) returns contents of ip_vmax as a double


% --- Executes during object creation, after setting all properties.
function ip_vmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ip_vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ip_amax_Callback(hObject, eventdata, handles)
% hObject    handle to ip_amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ip_amax as text
%        str2double(get(hObject,'String')) returns contents of ip_amax as a double


% --- Executes during object creation, after setting all properties.
function ip_amax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ip_amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_pid.
function load_pid_Callback(hObject, eventdata, handles)
% hObject    handle to load_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myScara
global the1_pre;
global the2_pre;
global d3_pre;
global the4_pre;
global t1 t2 t3 t4

PID_Plot(handles)

x = str2double(get(handles.x_pid,'String'));
y = str2double(get(handles.y_pid,'String'));
z = str2double(get(handles.z_pid,'String'));
yaw = str2double(get(handles.yaw_pid,'String'));

[the1,the2,d3,the4] = myScara.InverseKinematic(myScara, x, y, z, yaw);
the1 = the1*180/pi;
the2 = the2*180/pi;
set(handles.edit1,'String',num2str(the1));
set(handles.edit2,'String',num2str(the2));
set(handles.edit3,'String',num2str(d3));
set(handles.edit4,'String',num2str(the4));

v_max = str2double(get(handles.ip_vmax,'String'));
a_max = str2double(get(handles.ip_amax,'String'));

if(handles.lspb.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Trapezoidal_Vel_Profile(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Trapezoidal_Vel_Profile(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Trapezoidal_Vel_Profile(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Trapezoidal_Vel_Profile(abs(the4-the4_pre),v_max,a_max);
elseif(handles.scurve.Value)
    [tt1,tt1_dot,tt1_2dot,t1] = Scurve5Segment(abs(the1-the1_pre),v_max,a_max);
    [tt2,tt2_dot,tt2_2dot,t2] = Scurve5Segment(abs(the2-the2_pre),v_max,a_max);
    [dd3,d3_dot,d3_2dot,t3] = Scurve5Segment(abs(d3-d3_pre),v_max/20,a_max);
    [tt4,tt4_dot,tt4_2dot,t4] = Scurve5Segment(abs(the4-the4_pre),v_max,a_max);    
end
plot(handles.t1_pid, t1, the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1, 'r-');
hold on
plot(handles.t2_pid, t2, the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2, 'r-');
hold on
plot(handles.d3_pid, t3, d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3, 'r-');
hold on
plot(handles.t4_pid, t4, the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4, 'r-');
hold on

load_system('PID_Controller');
open_system('PID_Controller'); 

set_param('PID_Controller/motor1/Integrator','InitialCondition',num2str(the1_pre));
set_param('PID_Controller/motor2/Integrator','InitialCondition',num2str(the2_pre));
set_param('PID_Controller/motor3/Integrator','InitialCondition',num2str(d3_pre));
set_param('PID_Controller/motor4/Integrator','InitialCondition',num2str(the4_pre));

max_value = max([t1(100), t2(100), t3(100), t4(100)]);
t_in = linspace(0, max_value, 100);
sig1 = [t_in;the1_pre+((the1-the1_pre)/abs(the1-the1_pre))*tt1];
sig2 = [t_in;the2_pre+((the2-the2_pre)/abs(the2-the2_pre))*tt2];
sig3 = [t_in;d3_pre+((d3-d3_pre)/abs(d3-d3_pre))*dd3];
sig4 = [t_in;the4_pre+((the4-the4_pre)/abs(the4-the4_pre))*tt4];

save theta1set.mat sig1;
save theta2set.mat sig2;
save d3set.mat sig3;
save theta4set.mat sig4;

set_param('PID_Controller','StopTime',num2str(t_in(end)));
%set_param('PID_Controller','FixedStep',num2str(t1(end)/99.5));
set_param('PID_Controller','SimulationCommand','start');

the1_pre = the1;
the2_pre = the2;
d3_pre = d3;
the4_pre = the4;

% --- Executes on button press in run_pid.
function run_pid_Callback(hObject, eventdata, handles)
% hObject    handle to run_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global t1 t2 t3 t4
persistent  tout theta1out theta2out d3out theta4out

tout = evalin('base','tout')';
theta1out = evalin('base','theta1out')';
theta1out = theta1out(1:length(tout)/100:end); % lay 100 mau de cung kich thuoc voi t1

theta2out = evalin('base','theta2out')';
theta2out = theta2out(1:length(tout)/100:end);

d3out = evalin('base','d3out')';
d3out = d3out(1:length(tout)/100:end);

theta4out = evalin('base','theta4out')';
theta4out = theta4out(1:length(tout)/100:end);


xx = [];
yy = [];
zz = [];

for i = 1:1:length(t1)
    myScara = SCARA(handles,theta1out(i),theta2out(i),d3out(i),theta4out(i));
    VeRobot(myScara,handles,20,30);
    xx(i) = myScara.pos(4,1);
    yy(i) = myScara.pos(4,2);
    zz(i) = myScara.pos(4,3);
    
    plot(handles.t1_pid, t1(1:i), theta1out(1,1:i), 'b-');
    plot(handles.t2_pid, t2(1:i), theta2out(1,1:i), 'b-');
    plot(handles.d3_pid, t3(1:i), d3out(1,1:i), 'b-');
    plot(handles.t4_pid, t4(1:i), theta4out(1,1:i), 'b-');
    pause(0.01);
end

plot3(handles.robot_plot, xx, yy, zz,'MarkerSize', 5, 'Color', 'red');

function x_pid_Callback(hObject, eventdata, handles)
% hObject    handle to x_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_pid as text
%        str2double(get(hObject,'String')) returns contents of x_pid as a double


% --- Executes during object creation, after setting all properties.
function x_pid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_pid_Callback(hObject, eventdata, handles)
% hObject    handle to y_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_pid as text
%        str2double(get(hObject,'String')) returns contents of y_pid as a double


% --- Executes during object creation, after setting all properties.
function y_pid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_pid_Callback(hObject, eventdata, handles)
% hObject    handle to z_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z_pid as text
%        str2double(get(hObject,'String')) returns contents of z_pid as a double


% --- Executes during object creation, after setting all properties.
function z_pid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw_pid_Callback(hObject, eventdata, handles)
% hObject    handle to yaw_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw_pid as text
%        str2double(get(hObject,'String')) returns contents of yaw_pid as a double


% --- Executes during object creation, after setting all properties.
function yaw_pid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw_pid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
