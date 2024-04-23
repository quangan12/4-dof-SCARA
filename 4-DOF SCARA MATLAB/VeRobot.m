function VeRobot(robot,handles,Az,El)
% Az la de dieu chinh huong quay quanh truc z
% El la de dieu chinh huong quay quanh truc y


robot_plot = handles.robot_plot;
cla(robot_plot,'reset')
hold(robot_plot,'on')
grid(robot_plot,'on')

p0 = [0 0 0];
p1 = robot.pos(1,:);    %vi tri cua khop 1
p2 = robot.pos(2,:);    %vi tri cua khop 2
p3 = robot.pos(3,:);    %vi tri cua khop 3
p4 = robot.pos(4,:);    %vi tri cua khop 4
the = robot.theta;


xlabel(robot_plot,'x');
ylabel(robot_plot,'y');
zlabel(robot_plot,'z');


% Ve base

%ve khoi hop duoi de
fill3(robot_plot,[-0.15 -0.15 0.15 0.15],[-0.15 0.15 0.15 -0.15],[0 0 0 0],'k')
fill3(robot_plot,[-0.15 -0.15 0.15 0.15],[-0.15 0.15 0.15 -0.15],[0.02 0.02 0.02 0.02],'k')
fill3(robot_plot,[-0.15 0.15 0.15 -0.15],[-0.15 -0.15 -0.15 -0.15],[0 0 0.02 0.02],'k')
fill3(robot_plot,[-0.15 0.15 0.15 -0.15],[0.15 0.15 0.15 0.15],[0 0 0.02 0.02],'k')
fill3(robot_plot,[0.15 0.15 0.15 0.15],[-0.15 0.15 0.15 -0.15],[0 0 0.02 0.02],'k')
fill3(robot_plot,[-0.15 -0.15 -0.15 -0.15],[-0.15 0.15 0.15 -0.15],[0 0 0.02 0.02],'k')

VeHinhTru(handles,0,0,0.02,0.12,0.159,[0, 0.5, 1])
% 
% %link 1
VeHinhTru(handles,p0(1), p0(2),0.14,0.1,0.04, [0, 0.7, 1]);

%tinh toan toa do 4 dinh tu do ve khoi tru
[x,y] = tinhtoadoxoay(p0(1), p0(2), 0.1, 0.2, the(1)); %nhap x,y, chieu rong, chieu dai mat day, goc theta
fill3(robot_plot,[x(1) x(2) x(3) x(4)],[y(1) y(2) y(3) y(4)],[0.14 0.14 0.14 0.14],[0, 0.7, 1]) %ve mat duoi
fill3(robot_plot,[x(1) x(2) x(3) x(4)],[y(1) y(2) y(3) y(4)],[0.18 0.18 0.18 0.18],[0, 0.7, 1]) %ve mat tren
fill3(robot_plot,[x(1) x(4) x(4) x(1)],[y(1) y(4) y(4) y(1)],[0.14 0.14 0.18 0.18],[0, 0.7, 1]) %ve mat ben
fill3(robot_plot,[x(2) x(3) x(3) x(2)],[y(2) y(3) y(3) y(2)],[0.14 0.14 0.18 0.18],[0, 0.7, 1]) %ve mat ben


% 
%link2
VeHinhTru(handles,p1(1),p1(2),0.14,0.1,0.08, [0, 0.55, 0]);
[x1,y1] = tinhtoadoxoay(p1(1), p1(2), 0.1, 0.3,the(1)+the(2));
fill3(robot_plot,[x1(1) x1(2) x1(3) x1(4)],[y1(1) y1(2) y1(3) y1(4)],[0.18 0.18 0.18 0.18],[0, 0.55, 0]) %ve mat duoi
fill3(robot_plot,[x1(1) x1(2) x1(3) x1(4)],[y1(1) y1(2) y1(3) y1(4)],[0.22 0.22 0.22 0.22],[0, 0.55, 0]) %ve mat tren
fill3(robot_plot,[x1(1) x1(4) x1(4) x1(1)],[y1(1) y1(4) y1(4) y1(1)],[0.18 0.18 0.22 0.22],[0, 0.55, 0]) %ve mat ben
fill3(robot_plot,[x1(2) x1(3) x1(3) x1(2)],[y1(2) y1(3) y1(3) y1(2)],[0.18 0.18 0.22 0.22],[0, 0.55, 0]) %ve mat ben

VeHinhTru(handles,p2(1),p2(2),0.18,0.1,0.04, [0, 0.55, 0]);

%
% %link3
VeHinhTru(handles,p2(1),p2(2),p3(3)+0.2,0.03,0.005,[0, 0.7, 0]);
VeHinhTru(handles,p2(1),p2(2),p3(3)-0.005,0.03,0.005,[0, 0.7, 0]);
VeHinhTru(handles,p3(1),p3(2),p3(3),0.02,0.2,[0 0 0]);


%gioi han do dai cac truc
xlim(robot_plot,[-1 1]);
ylim(robot_plot,[-1 1]);
zlim(robot_plot,[0 0.5]);
view(robot_plot,Az,El)
grid on;
rotate3d('on');
%axis(robot_plot,'equal')