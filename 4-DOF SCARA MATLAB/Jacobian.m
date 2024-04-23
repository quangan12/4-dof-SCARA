function J = Jacobian(obj)

a = obj.a;
alpha = obj.alpha;
d = obj.d;
theta = obj.theta;

A01 = Matran_A(a(1),alpha(1),d(1),theta(1));
A12 = Matran_A(a(2),alpha(2),d(2),theta(2));
A23 = Matran_A(a(3),alpha(3),d(3),theta(3));
A34 = Matran_A(a(4),alpha(4),d(4),theta(4));

A02 = A01*A12;
A03 = A01*A12*A23;
A04 = A01*A12*A23*A34;

z0 = [0 0 1]';
p0 = [0 0 0]';

z1 = A01(1:3,3);
z2 = A02(1:3,3);
z3 = A03(1:3,3);

p1 = A01(1:3,4);
p2 = A02(1:3,4);
p3 = A03(1:3,4);
p4 = A04(1:3,4);

%% Ma tran Jacobian
J = [cross(z0,p4-p0) cross(z1,p4-p1) z2 cross(z3,p4-p3);
                 z0               z1  0               z3];
end