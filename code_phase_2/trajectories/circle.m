function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
total_time=8;
% the Quard will fly in a circle (2*pi), form the angle perspective of view
% it will accelerate in first half of total time to get pi, and decelerate 
% in the second half time to get 2*pi
angle_acc=(2*pi)/(total_time/2)^2;
dt=0.01;

   if t>total_time
    pos = [5; 0; 2.5];
    vel = [0;0;0];
    acc = [0;0;0];
   else 
       
   angle=get_agnle(angle_acc,total_time,t);
   pos=[5*cos(angle);5*sin(angle);2.5*angle/(2*pi)];
   vel= get_vel(t);
   acc = (get_vel(t+dt) - get_vel(t))/dt;
   end
   
   yaw=0;
   yawdot=0;
   
function [angle]=get_agnle(angle_acc,total_time,t)
   if t>=0&&t<=total_time/2
   % angle_acc=(2*pi)/(total_time/2)^2;
   % angle_vel=angle_acc*t;
     angle=0.5*angle_acc*t^2;
    elseif t>total_time/2
    v0=angle_acc*total_time/2;
   % angle_vel=v0-angle_acc*t;
     angle=pi+v0*(t-total_time/2)-0.5*angle_acc*(t-total_time/2)^2;
    end
end

  function vel = get_vel(t)
        angle1 = get_agnle(angle_acc,total_time,t);
        pos1=[5*cos(angle1);5*sin(angle1);2.5*angle1/(2*pi)];
        angle2 = get_agnle(angle_acc,total_time,t+dt);
        pos2=[5*cos(angle2);5*sin(angle2);2.5*angle2/(2*pi)];
        vel = (pos2 - pos1)/dt;
    end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
