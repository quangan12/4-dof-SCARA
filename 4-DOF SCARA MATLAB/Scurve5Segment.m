function [q,v,a,t] = Scurve5Segment(q_max,v_max,a_max)
if v_max > sqrt(q_max*a_max/2)
   v_max = sqrt(q_max*a_max/2);
end
    
t1 = v_max/a_max;
t2 = 2*t1;
t3 = q_max/v_max;
t4 = t3 + t1;
tf = t3 + t2;
j = a_max/t1;

t = linspace(0, tf, 100);
q = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));
    
for i = 1:length(t)
   if t(i) <= t1
       a(i) = j*t(i);
       v(i) = j*t(i)^2/2;
       q(i) = j*t(i)^3/6;
       
   elseif t(i) <= t2 
       a(i) = a_max - j*(t(i)-t1);
       v(i) = j*t1^2/2 + a_max*(t(i)-t1) - j*(t(i)-t1)^2/2;
       q(i) = j*t1^3/6 + j*t1^2/2*(t(i)-t1) + a_max*(t(i)-t1)^2/2 - j*(t(i)-t1)^3/6;
   elseif t(i) <= t3
       a(i) = 0;
       v(i) = v_max;
       q(i) = a_max*t1^2 + v_max*(t(i)-t2);
   elseif t(i) <= t4
       a(i) = -j*(t(i)-t3);
       v(i) = v_max - j*(t(i)-t3)^2/2;
       q(i) = a_max*t1^2 + v_max*(t3-t2) + v_max*(t(i)-t3) - j*(t(i)-t3)^3/6;
   elseif t(i) <= tf
       a(i) = -a_max + j*(t(i)-t4);
       v(i) = v_max - j*(t4-t3)^2/2 - a_max*(t(i)-t4) + j*(t(i)-t4)^2/2;
       q(i) = q_max - j*(tf - t(i))^3/6;  
   end
end
end