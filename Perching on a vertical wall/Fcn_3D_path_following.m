function desired_accelerations=Fcn_3D_path_following(in)
%% allocate input
r=in(1:3);
r_dot=in(4:6);
t=in(7);
%% kp kd
kp=[10 10 10]' ; % note [kpx kpy kpz]
kd=[5 5 5]';
%% desired path
r_desired=[2.5*t-4.835 0 1.4*t-2.607]' ;
r_desired_dot=[2.5 0 1.4]' ;
r_desired_dot_dot=[0 0 0]' ;
 n=[0 1 0]' ; % unit normal vector n
 b=[-1.4/sqrt(2.5^2+1.4^2) 0 2.5/sqrt(2.5^2+1.4^2)]' ; % unit tagent vector b
%% output desired acceleration
ep=dot(r_desired-r,n)*n+dot(r_desired-r,b)*b;
ev=r_desired_dot-r_dot;
desired_accelerations=kp.*ep+kd.*ev;
%% sources for paths 
%% r_desired=[t sin(t) 0]'
% r_desired_dot=[1 cos(t) 0]'
% r_desired_dot_dot=[0 -sin(t) 0]
% n=[cos(t)/sqrt(1+cos(t)^2) -1/sqrt(1+cos(t)^2) 0]' % unit normal vector n
% b=[0 0 -1]'                                        % unit binormal vector
%% r_desired=[0.05*t^2 0 0]'
% r_desired_dot=[0.1*t 0 0]'
% r_desired_dot_dot=[0.1 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b 
%%  r_desired=[3*t 0 0]'
% r_desired_dot=[3 0 0]'
% r_desired_dot_dot=[0 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b
%% 3 pieces
% if t<1/3
% r_desired=[0.05*(1/3)^2 0 0]'
% r_desired_dot=[0.1*1/3 0 0]'
% r_desired_dot_dot=[0.1 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b
% elseif (1/3)<=t & t<(2/3)
% r_desired=[0.05*(2/3)^2 0 0]'
% r_desired_dot=[0.1*(2/3) 0 0]'
% r_desired_dot_dot=[0.1 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b
% elseif (2/3)<=t & t<=1
% r_desired=[0.05*1^2 0 0]'
% r_desired_dot=[0.1*1 0 0]'
% r_desired_dot_dot=[0.1 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b
% elseif 1<t & t<=2
% r_desired=[0.05+0.1*t 0 0]'
% r_desired_dot=[0.1 0 0]'
% r_desired_dot_dot=[0 0 0]'
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b    
% end
%% 
% if t<10
% r_desired=[1/2*3.41855*t^2 0 1/2*(-2)*t^2]'
% r_desired_dot=[3.41855*t 0 -2*t]'
% r_desired_dot_dot=[3.41855 0 -2]'
% elseif 10<=t & t<=15
% r_desired=[1/2*3.41855*10^2+34.1855*(t-10) 0 1/2*(-2)*10^2-20*(t-10)]'
% r_desired_dot=[34.1855 0 -20]'
% r_desired_dot_dot=[0 0 0]'
% end
% n=[20 0 34.1855]' /sqrt(20^2+34.1855^2)% unit normal vector n
% b=[0 -1 0]' % unit tagent vector b 
%% 
% if t<1
% r_desired=[1/2*3*t^2 0 0]'
% r_desired_dot=[3*t 0 0]'
% r_desired_dot_dot=[3 0 0]'
% else
% r_desired=[1/2*3*1^2+3*(t-1) 0 0]'
% r_desired_dot=[3 0 0]'
% r_desired_dot_dot=[0 0 0]'  
% end
% n=[0 1 0]' % unit normal vector n
% b=[0 0 1]' % unit tagent vector b 
%%
%T=10;n=25
%for i=1:n
%    if (i-1)*T/n<=t & t<i*T/n
%        r_desired=[1/2*3.41855*(i*T/n)^2 0 1/2*(-2)*(i*T/n)^2]'
%r_desired_dot=[3.41855*(i*T/n) 0 -2*(i*T/n)]'
%    end
%end
%if 10<=t & t<=15
%r_desired=[1/2*3.41855*10^2+34.1855*(t-10) 0 1/2*(-2)*10^2-20*(t-10)]'
%r_desired_dot=[34.1855 0 -20]'
%end
%r_desired_dot_dot=[3.41855 0 -2]'
%n=[20 0 34.1855]' /sqrt(20^2+34.1855^2)% unit normal vector n
%b=[0 -1 0]' % unit tagent vector b 