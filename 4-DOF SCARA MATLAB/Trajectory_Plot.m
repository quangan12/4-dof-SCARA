function Trajectory_Plot(handles)
%% the1 the2 d3 the4
axes(handles.t1)
cla
hold on
grid on
title('${\theta_1}$ (deg)', 'Interpreter','latex')

axes(handles.t2)
cla
hold on
grid on
title('${\theta_2}$ (deg)', 'Interpreter','latex')

axes(handles.d3)
cla
hold on
grid on
title('${d_3}$ (m)', 'Interpreter','latex')

axes(handles.t4)
cla
hold on
grid on
title('${\theta_4}$ (deg)', 'Interpreter','latex')
xlabel('Time (s)');

%% the1dot the2dot d3dot the4dot
axes(handles.t1_dot)
cla
hold on
grid on
title('$\dot{\theta_1}$ (deg/s)', 'Interpreter','latex')

axes(handles.t2_dot)
cla
hold on
grid on
title('$\dot{\theta_2}$ (deg/s)', 'Interpreter','latex')

axes(handles.d3_dot)
cla
hold on
grid on
title('$\dot{d_3}$ (m/s)', 'Interpreter','latex')

axes(handles.t4_dot)
cla
hold on
grid on
title('$\dot{\theta_4}$ (deg/s)', 'Interpreter','latex')
xlabel('Time (s)');

%% 2 dot 

axes(handles.t1_2dot)
cla
hold on
grid on
title('$\stackrel{..}{\theta_1}$ (deg/s2)', 'Interpreter','latex')

axes(handles.t2_2dot)
cla
hold on
grid on
title('$\stackrel{..}{\theta_2}$ (deg/s2)', 'Interpreter','latex')

axes(handles.d3_2dot)
cla
hold on
grid on
title('$\stackrel{..}{d_3}$ (m/s2)', 'Interpreter','latex')

axes(handles.t4_2dot)
cla
hold on
grid on
title('$\stackrel{..}{\theta_4}$ (deg/s2)', 'Interpreter','latex')
xlabel('Time (s)');
end