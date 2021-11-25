function out=Fcn_references_and_parameters(t)
%% specify parameters
m=3;
g=9.81;
l=0.225;
Ix=4.856*10^(-3);
Iy=4.856*10^(-3);
Iz=8.801*10^(-3);
k=2.980*10^-6;
c=1.140*10^-7;

%% specify your references here
r_desired=[-3 0 0.6]' ;
r_desired_dot=[0 0 0]' ;
psi_desired=0 ;

%% output
out=[m;g;l;Ix;Iy;Iz;k;c;
    r_desired;
    r_desired_dot;
    psi_desired];
end
