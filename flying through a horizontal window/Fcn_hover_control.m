function out=Fcn_hover_control(in)
%% kp,kd
kp=[2 2 2]'; % kp=[kpx kpy kpz]
kd=[2 2 2]'; % kd=[kdx kdy kdz]
%% allocate inputs
m=in(1);
g=in(2);
r_desired=in(9:11);
r_desired_dot=in(12:14);
psi_desired=in(15);
r=in(16:18);
r_dot=in(19:21);
%% desired accelerations
a_desired=kp.*(r_desired-r)+kd.*(r_desired_dot-r_dot);
ax_desired=a_desired(1);
ay_desired=a_desired(2);
az_desired=a_desired(3);
%% desired roll and pitch angle & commanded thrust f
phi_desired=1/g*(sin(psi_desired)*ax_desired-cos(psi_desired)*ay_desired);
theta_desired=1/g*(cos(psi_desired)*ax_desired+sin(psi_desired)*ay_desired);
f=m*(az_desired+g);
%% output 
out=[f;phi_desired;theta_desired;psi_desired] ; % output is a 12 by 1 vector