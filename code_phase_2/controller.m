function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% position and attitude controller params
Kp = [30;30;95];
Kd = [20;20;18];

KpM = ones(3,1)*2000;
KdM = ones(3,1)*200;

acc_des = qd{qn}.acc_des + Kd.*(qd{qn}.vel_des - qd{qn}.vel) + Kp.*(qd{qn}.pos_des - qd{qn}.pos);

% Desired roll, pitch and yaw
phi_des = 1/params.grav * (acc_des(1)*sin(qd{qn}.yaw_des) - acc_des(2)*cos(qd{qn}.yaw_des));
theta_des = 1/params.grav * (acc_des(1)*cos(qd{qn}.yaw_des) + acc_des(2)*sin(qd{qn}.yaw_des));
psi_des = qd{qn}.yaw_des;

pqr_des = [0;0; qd{qn}.yawdot_des];
euler_des = [phi_des;theta_des;psi_des];
% Thurst
qd{qn}.acc_des(3);
F  = params.mass*(params.grav + acc_des(3));
% Moment
M =  params.I*(KdM.*(pqr_des - qd{qn}.omega) + KpM.*(euler_des - qd{qn}.euler));
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
