 classdef SCARA
    
    properties
        %% DH parameter
        a;
        d;
        alpha;  % unit deg
        theta;  % unit deg
        
        %% Limatation of workspace
        pos;    %%position of 4 joints 1 2 3 4
        orien;  %%orientation of 4 joints 1 2 3 4, unit radian
%         %% thong so tinh dong hoc nguoc
%         x;
%         y;
%         z;
%         yaw;
%         theta1; theta2; d3; theta4;
    end
    
    methods(Static)
        %nhap cac thong so cua robot
        function obj = SCARA(handles, the1, the2, d3, the4)
            obj.a = [0.2; 0.3; 0; 0];
            obj.d = [0.179; 0; 0; 0];
            obj.alpha = [0.00; 0.00; 0.00; 180];
            obj.theta = [0.00; 0.00; 0.00; 0.00];
            obj.d(3)=d3;
            obj.theta(1)=the1;
            obj.theta(2)=the2;
            obj.theta(4)=the4;
            [obj.pos,obj.orien] = obj.ForwardKinematic(obj);
            
        end
        %
        
        %%
        function [p_robot,o_robot] = ForwardKinematic(self)
            a = self.a;
            alpha = self.alpha*pi/180;
            d = self.d;
            theta = self.theta*pi/180;
            
            %% Ham tinh dong hoc thuan cua robot
            % Input: DH Parameter
            % Output: joint position p1 p2 p3 p4     (x y z)
            %         joint orientation o1 o2 o3 o4  (roll pitch yaw)
            %% FK Matrix
            A0_1 = Matran_A(a(1),alpha(1),d(1),theta(1)) ;
            A1_2 = Matran_A(a(2),alpha(2),d(2),theta(2)) ;
            A2_3 = Matran_A(a(3),alpha(3),d(3),theta(3)) ;
            A3_4 = Matran_A(a(4),alpha(4),d(4),theta(4)) ;

            A0_2=A0_1*A1_2;
            A0_3=A0_1*A1_2*A2_3;
            A0_4=A0_1*A1_2*A2_3*A3_4;   % Te

            p0 = [0;0;0];
            [p1, o1] = t_pose(A0_1,p0);
            [p2, o2] = t_pose(A0_2,p0);
            [p3, o3] = t_pose(A0_3,p0);
            [p4, o4] = t_pose(A0_4,p0);

            p_robot = [p1 p2 p3 p4]';
            o_robot = [o1; o2; o3; o4];
        end
        %%
        function [the1, the2, d3, the4] = InverseKinematic(self, x, y, z, yaw)
            a = self.a;
            d = self.d; 
            % Tinh dong hoc nguoc cua robot
            % Input: x, y, z, yaw
            % Output: theta1, theta2, d3, theta4
            
%             % xe = a1.c1+a2.c12
%             % ye = a1.s1+a2.s12
%             % ze = d1+d3+d4
%             d3 = z - d(1) - d(4);
%             the2 = acos((x^2 + y^2 - (a(1)^2 + a(2)^2))/(2*a(1)*a(2)));
%             %cos(the1) = (a(1)*x + a(2)*(x*cos(the2) + y*sin(the2)))/(x^2 + y^2);
%             %sin(the1) = (a(1)*y + a(2)*(y*cos(the2) - x*sin(the2)))/(x^2 + y^2);
%             the1 = atan(((a(1)*y + a(2)*(y*cos(the2) - x*sin(the2))))/((x^2 + y^2)/(a(1)*x + a(2)*(x*cos(the2) + y*sin(the2)))/(x^2 + y^2)));
%             the4 = yaw;
            c2 = (x^2 + y^2 - a(1)^2 - a(2)^2)/(2*a(1)*a(2));
            if (abs(c2)<=1)
                s2 = sqrt(1-c2^2);
                theta21 = atan2(s2,c2);
                theta22 = atan2(-s2,c2);
                
                if abs(theta21 - pi/2) < pi
                    theta2 = theta21;
                else
                    theta2 = theta22;
                    s2 = -s2;
                end
                
                t1 = [a(1)+a(2)*c2 -a(2)*s2;a(2)*s2 a(1)+a(2)*c2]^(-1)*[x;y];
                c1 = t1(1);
                s1 = t1(2);
                theta1 = atan2(s1,c1);

                dd3 = z - d(1);
                theta4 = yaw - ( theta1*180/pi + theta2*180/pi);
                
                if (abs(theta1*180/pi)>125)||(abs(theta2*180/pi)>145)||(dd3<-0.15)
                    questdlg('Nhap dung thong so','Warning','OK','OK');
                else
                    the1 = theta1;
                    the2 = theta2;
                    d3 = dd3;
                    the4 = theta4;
                end
            else
                questdlg('Nhap dung thong so','Warning','OK','OK');
            end
        end
    end
end

