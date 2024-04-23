function [X,Y] = tinhtoadoxoay(x,y,r,a,theta)
% ham nay de tinh toa do 4 diem goc cua hinh chu nhat khi xong tam 1 canh
% quanh theta
the = theta*pi/180;
%diem goc thu nhat
X(1) = x + r*cos(-pi/2 + the);
Y(1) = y + r*sin(-pi/2 + the);

%diem goc thu hai
X(2) = x + r*cos(pi/2 + the);
Y(2) = y + r*sin(pi/2 + the);

%diem goc thu ba
X(3) = x + (sqrt(r^2 + a^2))*cos(atan(r/a)+the);
Y(3) = y + (sqrt(r^2 + a^2))*sin(atan(r/a)+the);

%diem goc thu tu
X(4) = x + (sqrt(r^2 + a^2))*cos(-atan(r/a)+the);
Y(4) = y + (sqrt(r^2 + a^2))*sin(-atan(r/a)+the);
end