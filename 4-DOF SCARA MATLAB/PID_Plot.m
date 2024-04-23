function PID_Plot(handles)
axes(handles.t1_pid)
cla
hold on
grid on
title('${\theta_1}$ (deg)', 'Interpreter','latex')
xlabel('Time (s)');

axes(handles.t2_pid)
cla
hold on
grid on
title('${\theta_2}$ (deg)', 'Interpreter','latex')
xlabel('Time (s)');

axes(handles.d3_pid)
cla
hold on
grid on
title('${d_3}$ (m)', 'Interpreter','latex')
xlabel('Time (s)');

axes(handles.t4_pid)
cla
hold on
grid on
title('${\theta_4}$ (deg)', 'Interpreter','latex')
xlabel('Time (s)');
end