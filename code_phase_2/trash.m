close all
clear
t = 0:0.01:2*pi;
st = 5+5*sin(t);
ct = -5+5*cos(t);
%z=0:0.01:2.5;
z=2.5*t/5;
%pos=[st;-ct;2.5*t/5];
figure()
plot3(st,-ct,2.5*t/5);
%%
clear 
close all
clc
t=10;
t_axis = 0:t/100:pi;
x=5*sin(t_axis);
y=5-5*cos(t_axis);
[~,t_size]=size(t_axis);
yawdot = 0;
pos =[x;y;t_axis/pi];
figure
plot3(x,y,2.5*t_axis/pi)
title('no1')
x_vel=5*cos(t_axis);
y_vel=x;
z_vel=1/pi;
vel = [x_vel; y_vel; z_vel];
x_acc=-x; y_acc=5*cos(t_axis);z_acc=zeros(1,t_size);
acc=[x_acc;y_acc;z_acc];
for i=1:t_size
    
    if t(i)<2*pi
        yaw(i)=t(i);
    else
        yaw(i)=t(i)-2*pi;
    end
end
%%
time=0;
for literation=1:50
x=5+5*sin(time);
y=5-5*cos(time);
z=0:0.01:2.5;
time=time+0.01;    
  
end
figure
plot3(x,y,z);  
%%
clear
syms c_7 c_6 c_5 c_4 c_3 c_2 c_1 c_0 t
x=c_7*t^7+c_6*t^6+c_5*t^5+c_4*t^4+c_3*t^3+c_2*t^2+c_1*t+c_0;
vel=diff(x,t);
acc=diff(vel,t);
t=0;
vel