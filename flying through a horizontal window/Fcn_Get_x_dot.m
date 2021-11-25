function x_dot=Fcn_Get_x_dot(in)
%% given parameters
m=3;%kg
M=diag([4.856, 4.856, 8.801])*10^(-3); % moment of inertia in the body frame
g=9.81 ;
e3=[0 0 1]';
 Ax=0.25;
 Ay=0.25;
 Az=0.25;
%% allocate the input 
x=in(1:12);
pn=x(1:3);
vn=x(4:6);
eta=x(7:9);
omegab=x(10:12);
f=in(13);
taub=in(14:16);
%% define matrix Rbn , Lbn and Omega
phi=eta(1);
theta=eta(2);
psi=eta(3);
Rbn=[cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
     cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
     -sin(theta)         sin(phi)*cos(theta)                            cos(phi)*cos(theta)                           ];
Lbn=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);
     0 cos(phi)            -sin(phi);
     0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
Omega=[0         -omegab(3) omegab(2);
       omegab(3) 0         -omegab(1);
      -omegab(2) omegab(1) 0];
%% nonlinear model: shown on the handout
pn_dot=vn;
vn_dot=1/m*Rbn*(f*e3)-g*e3; % -1/m*[Ax 0 0;0 Ay 0;0 0 Az]*pn_dot;    % pay attention to whether I need Ax, Ay, Az
eta_dot=Lbn*omegab;
omegab_dot=inv(M)*(taub-Omega*M*omegab);
%% output x_dot
x_dot=[pn_dot;
       vn_dot;
       eta_dot;
       omegab_dot];
