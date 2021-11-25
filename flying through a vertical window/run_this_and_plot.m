clear all;close all
x_0=[-3.450 -4.669 -2.483, 0 0 0, 0 0 0, 0 0 0]'; %  specify your initial conditon
m=3;
g=9.81;
l=0.225 ; % 1 m this is not a correct length, temporarily for convenience,the same thing as rotors 
Ix=4.856*10^(-3);
Iy=4.856*10^(-3);
Iz=8.801*10^(-3);
k=2.980*10^-6;
c=1.140*10^-7;
 sim('phase2_3D_path_following')
 sim('phase3_attitude_control')
 sim('phase4_attitude_control')
 sim('phase5_hover_control')

%% plot
fontsize=15;
markersize=10;
% data
step=1;
x_all=[x_phase2(1:end-1,:);x_phase3(1:end-1,:);x_phase4(1:end-1,:);x_phase5];   % pay attention
% source: x_all=[x_phase2(1:end-1,:);x_phase3(1:end-1,:);x_phase4(1:end-1,:);x_phase5]; 
n=numel(x_all(:,1)); % number of samples  
time_all=(0:0.01:(n-1)*0.01)';                       % pay attention: source:(n-1)*0.01
dataNum=max(size(time_all));
pn_all=x_all(1:step:end,1:3);
vn_all=x_all(1:step:end,4:6);
eta_all=x_all(1:step:end,7:9);
omega_all=x_all(1:step:end,10:12);
for i=1:n                                 % pay attention
  phi=eta_all(i,1);theta=eta_all(i,2);psi=eta_all(i,3);
  Rbn=[cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
     cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
     -sin(theta)         sin(phi)*cos(theta)                            cos(phi)*cos(theta)                           ];
  p_rotor1(i,:)=Rbn*[l 0 0]'+pn_all(i,:)';
  p_rotor2(i,:)=Rbn*[0 l 0]'+pn_all(i,:)';
  p_rotor3(i,:)=Rbn*[-l 0 0]'+pn_all(i,:)';
  p_rotor4(i,:)=Rbn*[0 -l 0]'+pn_all(i,:)';
  p_z(i,:)=Rbn*[0 0 l]'+pn_all(i,:)';
end
    
% trajectory of center of mass
figure;
hold on; box on; axis equal
set(get(gca,'title'), 'string', 'Trajectory', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'x (m)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', fontsize);
set(get(gca, 'zlabel'), 'String', 'z (m)', 'fontSize', fontsize);
plot3(pn_all(:,1),pn_all(:,2),pn_all(:,3),'k--');
view([20,20])
plot3(pn_all(1,1),pn_all(1,2),pn_all(1,3), 'marker', 'o', 'markersize', markersize); % initial position
plot3(pn_all(end,1),pn_all(end,2),pn_all(end,3), 'marker', 'o', 'markersize', markersize, 'MarkerFaceColor', 'b'); % final position
set(gca,'xlim',get(gca,'xlim')+[-1,1])
set(gca,'ylim',get(gca,'ylim')+[-1,1])
set(gca,'zlim',get(gca,'zlim')+[-1,1])

%  plot drones on the trajectories at different times 
for i=[1:50:201 231 281 381 481 1281]                  % pay attention
plot3([p_rotor1(i,1) p_rotor3(i,1)],[p_rotor1(i,2) p_rotor3(i,2)],[p_rotor1(i,3) p_rotor3(i,3)],'r')
plot3(p_rotor1(i,1),p_rotor1(i,2),p_rotor1(i,3),'r*')
plot3(p_rotor3(i,1),p_rotor3(i,2),p_rotor3(i,3),'rx')
plot3([p_rotor2(i,1) p_rotor4(i,1)],[p_rotor2(i,2) p_rotor4(i,2)],[p_rotor2(i,3) p_rotor4(i,3)],'g')
plot3(p_rotor2(i,1),p_rotor2(i,2),p_rotor2(i,3),'g*')
plot3(p_rotor4(i,1),p_rotor4(i,2),p_rotor4(i,3),'gx')
plot3([pn_all(i,1) p_z(i,1)],[pn_all(i,2) p_z(i,2)],[pn_all(i,3) p_z(i,3)],'b')

end


% horizontal window 
% plot3([-0.2 0.2],[-0.6 -0.6],[0 0],'k-','LineWidth',3)
% plot3([-0.2 0.2],[0.6 0.6],[0 0],'k-','LineWidth',3)
% plot3([-0.2 -0.2],[-0.6 0.6],[0 0],'k-','LineWidth',3)
% plot3([0.2 0.2],[-0.6 0.6],[0 0],'k-','LineWidth',3)
 % vertical window
 plot3([-0.2 0.2],[0 0],[-0.6 -0.6],'k-','LineWidth',3)
 plot3([-0.2 0.2],[0 0],[0.6 0.6],'k-','LineWidth',3)
 plot3([-0.2 -0.2],[0 0],[-0.6 0.6],'k-','LineWidth',3)
 plot3([0.2 0.2],[0 0],[-0.6 0.6],'k-','LineWidth',3)
 % vertical wall
% plot3([0 0],[-1 1],[-1 -1],'k-','LineWidth',2)
% plot3([0 0],[-1 1],[1 1],'k-','LineWidth',2)
% plot3([0 0],[-1 -1],[-1 1],'k-','LineWidth',2)
% plot3([0 0],[1 1],[-1 1],'k-','LineWidth',2)
 


% trajectory and attitude animation
xlim=get(gca,'xlim'); xwidth=max(xlim)-min(xlim);
ylim=get(gca,'ylim'); ywidth=max(ylim)-min(ylim);
zlim=get(gca,'zlim'); zwidth=max(zlim)-min(zlim);
scale=1.25*l% 1.25 was got from my own test. I have no idea about the relationship.Shiyu: scale=1/8*min([xwidth,ywidth,zwidth])
hObject=fcn_QuadcopterAnimation(-1, x_0(1:3), fcn_Euler2Rotation(x_0(7), x_0(8), x_0(9)), scale); % create object
step=ceil(dataNum/100);
for i=1:step:dataNum
    pni=pn_all(i,:)';
    etai=eta_all(i,:)';
    Ri=fcn_Euler2Rotation(etai(1), etai(2), etai(3))
    fcn_QuadcopterAnimation(hObject, pni, Ri, scale);
    pause(0.1)
    disp(strcat(num2str(i/dataNum*100, '%3.0f'),'% animation'))
end


% Position x-y-z
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Position', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'x (m)', 'fontSize', fontsize);
plot(time_all,pn_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', fontsize);
plot(time_all,pn_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'z (m)', 'fontSize', fontsize);
plot(time_all,pn_all(:,3));

% velocity
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Linear velocity', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'v_x (m/s)', 'fontSize', fontsize);
plot(time_all,vn_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'v_y (m/s)', 'fontSize', fontsize);
plot(time_all,vn_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'v_z (m/s)', 'fontSize', fontsize);
plot(time_all,vn_all(:,3));

% Euler angles
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Euler angles', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\phi (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,1)/pi*180);
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\theta (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,2)/pi*180);
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', '\psi (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,3)/pi*180);

% omega
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Angular velocity', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\omega_x (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\omega_y (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', '\omega_z (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,3));

%% window parallel to the ground
% plot3(linspace(7.43,7.83,11),0.6*ones(1,11),-5.6*ones(1,11))
% plot3(linspace(7.43,7.83,11),-0.6*ones(1,11),-5.6*ones(1,11))
% plot3(7.43*ones(1,11),linspace(-0.6,0.6,11),-5.6*ones(1,11))
% plot3(7.83*ones(1,11),linspace(-0.6,0.6,11),-5.6*ones(1,11))
%% window vertical to the ground
% plot3(linspace(7.43,7.83,11),4.3*ones(1,11),0.59*ones(1,11))
% plot3(linspace(7.43,7.83,11),4.3*ones(1,11),-0.61*ones(1,11))
% plot3(7.43*ones(1,11),4.3*ones(1,11),linspace(-0.61,0.59,11))
% plot3(7.83*ones(1,11),4.3*ones(1,11),linspace(-0.61,0.59,11))
%% vertical wall
% plot3(7.6*ones(1,11),linspace(-1,1,11),-ones(1,11))
% plot3(7.6*ones(1,11),linspace(-1,1,11),ones(1,11))
% plot3(7.6*ones(1,11),ones(1,11),linspace(-1,1,11))
% plot3(7.6*ones(1,11),-ones(1,11),linspace(-1,1,11))
%% trajectories of 4 rotors
% plot3(p_rotor1(:,1),p_rotor1(:,2),p_rotor1(:,3),'r');
% plot3(p_rotor2(:,1),p_rotor2(:,2),p_rotor2(:,3),'g');
% plot3(p_rotor3(:,1),p_rotor3(:,2),p_rotor3(:,3));
% plot3(p_rotor4(:,1),p_rotor4(:,2),p_rotor4(:,3));




