function [q,v,a,t] = Trapezoidal_Vel_Profile(q_max,v_max,a_max)
%% gia tri lon nhat co the cua v
if v_max > sqrt(q_max*a_max)
    v_max = sqrt(q_max*a_max);
end

t1 = v_max/a_max;
q1 = 1/2*a_max*t1^2;
t2 = (q_max-2*q1)/v_max + t1;
tf = t1+t2;

t = linspace(0, tf, 100);
q = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));
for i=1:length(t)
    if t(i)<t1
        q(i)=1/2*a_max*t(i)^2;
        v(i) = a_max*t(i);
        a(i) = a_max;
    elseif t(i)<t2
        q(i) = q1 + v_max*(t(i)-t1);
        v(i) = v_max;
        a(i) = 0;
    else
        q(i) = q_max - 1/2*a_max*(tf-t(i))^2;
        v(i) = a_max*(tf-t(i));
        a(i) = -a_max;
    end
end
