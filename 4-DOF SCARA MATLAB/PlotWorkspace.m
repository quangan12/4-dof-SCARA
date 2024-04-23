function PlotWorkspace(robot,handles)
th = deg2rad(linspace(-125, 125, 100));
X = 0.5*cos(th);
Y = 0.5*sin(th);
Z1 = ones(1, size(th, 2))*0;
Z2 = ones(1, size(th, 2))*0.179;
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [1 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

th = deg2rad(linspace(125, 245, 100));
X = 0.2*cos(125*pi/180) + 0.3*cos(th);
Y = 0.2*sin(125*pi/180) + 0.3*sin(th);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [1 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

th = deg2rad(linspace(-245, -125, 100));
X = 0.2*cos(-125*pi/180) + 0.3*cos(th);
Y = 0.2*sin(-125*pi/180) + 0.3*sin(th);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [1 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

R = sqrt(0.2^2 + 0.3^2 - 2*0.2*0.3*cosd(180-145));
th = deg2rad(linspace(-130, 130, 100));
X = R*cos(th);
Y = R*sin(th);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [0 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

X = linspace((0.2*cos(125*pi/180) + 0.3*cos(245*pi/180)),(R*cos(-130*pi/180)),100);
Y = ones(1, 100)*R*sin(-130*pi/180);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [0 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

X = linspace((0.2*cos(125*pi/180) + 0.3*cos(245*pi/180)),(R*cos(-130*pi/180)),100);
Y = ones(1, 100)*R*sin(130*pi/180);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [0 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

X = ones(1, 100)*(0.2*cos(125*pi/180) + 0.3*cos(245*pi/180));
Y = linspace(R*sin(-130*pi/180), R*sin(130*pi/180), 100);
surf([X;X], [Y;Y], [Z1;Z2], 'FaceColor', [0 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

end