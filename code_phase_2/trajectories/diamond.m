function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
total_t=8;
dt=0.01;

if t>total_t
       pos = [1;0;0];
       vel = [0;0;0];
       acc = [0;0;0];
else
     [pos, vel] = generate_pos_vel(total_t,t);
     [~, vel1] = generate_pos_vel(total_t,t+dt);
     acc = (vel1-vel)/dt;
end

yaw=0;
yawdot=0;
    function [pos, vel]=generate_pos_vel(total_t,t)
        if t >= 0 && t < total_t/4
           [pos, vel] = generate_trajectory([0;0;0], [0;1;1], total_t/4, t);
        elseif t >= total_t/4 && t < total_t/2
           [pos, vel] = generate_trajectory([0;1;1], [0;0;2], total_t/4, t-total_t/4);
        elseif t >= total_t/2 && t < total_t*3/4
           [pos, vel] = generate_trajectory([0;0;2], [0;-1;1], total_t/4, t-total_t/2);
        else
           [pos, vel] = generate_trajectory([0;-1;1], [1;0;0], total_t/4, t-total_t*3/4);
        end
    end 


%brief Bascily it accelerates at the first half of time, and decellerate at
%the second half.
    function [traj_pos,traj_vel]=generate_trajectory(startP,endP,total_t,t)
         traj_acc=(endP-startP)/(total_t/2)^2;
         if  t>=0 && t<= total_t/2
             traj_vel=traj_acc*t;
             traj_pos=startP+0.5*traj_acc*t^2;
         elseif t>total_t/2
             v0=traj_acc*total_t/2;
             traj_vel=v0-traj_acc*(t-total_t/2);
             traj_pos=startP+0.5*traj_acc*(total_t/2)^2+v0*(t-total_t/2)-0.5*traj_acc*(t-total_t/2)^2;
         end
    end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
