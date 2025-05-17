clc;clear;

%% Model Parameters
T = 100; % [s] run time
mtr_select = 0; % 1 - for linear motor model
                % 0 - for first order motor model

%% Controller Gains
lambda_p = 5;       lambda_y = 1;
D_p = 0*1.1;        D_y = 0*1.1;
eta_p = 1;        eta_y = 0.1;
phi_para_p = 0.01;   phi_para_y = 0.1;

control_gains = [lambda_p lambda_y;
                 D_p D_y;
                 eta_p eta_y;
                 phi_para_p phi_para_y];

%% Pitch Parameters
g = 9.81; % [m/s^2]
m = 0.80+0.089+0.089; % [kg]
kFF = 1.25e-6; % [unitless]
dF = 0.2; % [m]
dcm = 0.00325; % [m]
Jp = 0.0219; % [kg-m^2]
kRT1 = 1e-5; % [unitless]
kRT2 = 1.25e-7; % [unitless]
Dp = 0.00711; % [V-s/rad]
kVF = 1/0.04222;
% kVF = 1; % turn off kv effects

%% Pitch haaaa
hp_hat = Jp/(kFF*dF*kVF^2);
ap1_hat = Dp/(kFF*dF*kVF^2);
ap2_hat = m*g*dcm/(kFF*dF*kVF^2);
ap3_hat = -kRT1/(kFF*dF*kVF^2);
ap4_hat = -kRT2/(kFF*dF*kVF^2);

%% Yaw Parameters
kFT1 = 1e-5; % [unitless]
kFT2 = 1.25e-7; % [unitless]
kRF = 1.25e-6; % [unitless]
dR = 0.2; %[m]
Dy = 0.0220; % [V-s/rad]
Jy = 0.0370; % [kg-m^2]
kVR = 1/0.04222; % [unitless]
% kVR = 1; % turn off kv effects

%% Yaw haaa
hy_hat = -Jy/(kRF*dR*kVR^2);
ay1_hat = -Dy/(kRF*dR*kVR^2);
ay2_hat = kFT1/(kRF*dR*kVR^2);
ay3_hat = kFT2/(kRF*dR*kVR^2);

%% Bundle Params
% params = [pitch params, yaw params]
params_aero = [kFF kRF; % NOT USED IN MODEL
               kRT1 kFT1;
               kRT2 kRT2;
               kVF kVR;
               dF dR;
               Jp Jy;
               Dp Dy;
               dcm 0;
               m 0;
               g 0];

params_haaaa = [hp_hat hy_hat; % USED FOR CONTROLLER AND PLANT
               ap1_hat ay1_hat;
               ap2_hat ay2_hat;
               ap3_hat ay3_hat;
               ap4_hat 0];

out=sim("model_Quadrotor6_NonAdaptiveV2.slx");

List=get(out);
for No=1:1:length(List)
    out(1,1).find(char(List(No,1)));
    assignin('base',char(List(No,1)),out(1,1).find(char(List(No,1))))
end

colors = [0.8500 0.3250 0.0980
          0.0000 0.4470 0.7410
          0.9290 0.6940 0.1250
          0.4940 0.1840 0.5560];

pitch = zeros(length(time(:,1)), 1);
pitchd = zeros(length(time(:,1)), 1);
yaw = zeros(length(time(:,1)), 1);
yawd = zeros(length(time(:,1)), 1);

pitch(:,1) = rad2deg(states(1,1,:));
pitchd(:,1) = rad2deg(states(2,1,:));
yaw(:,1) = rad2deg(states(3,1,:));
yawd(:,1) = rad2deg(states(4,1,:));

%% Actual Vs Desired Plot (Combined)
figure(1);clf(1);
colororder(colors)
hold on
plot(time,rad2deg(des(:,1)),'r--','LineWidth',2)
plot(time,rad2deg(des(:,4)),'b--','LineWidth',2)
plot(time,pitch(:,1),'r','LineWidth',1)
plot(time,yaw(:,1),'b','LineWidth',1)
hold off
title('Actual and Desired Position')
xlabel('$time$ (s)','Interpreter','Latex')
ylabel('$\theta \phi (degrees)$','Interpreter','Latex')
legend('Desired Pitch','Desired Yaw','Actual Pitch','Actual Yaw','Interpreter','Latex')
grid on
% box on
% set(gca,'FontSize',18)
% axis([-0.6 0.6 -0.6 0.6 -0.6 0.6])
axis square

%% Pitch and Yaw States vs Desired (Individually)
figure(2);clf(2);
colororder(colors)
subplot(2,1,1)
hold on
plot(time,rad2deg(des(:,1)),'--','LineWidth',2)
plot(time,pitch(:,1),'b','LineWidth',1)
hold off
title('Pitch Actual/Desired v Time')
xlabel('$t$ (s)','Interpreter','Latex')
ylabel('$Pitch (degrees)$','Interpreter','Latex')
legend('$Pitch$ Desired','$Pitch$ Actual','Interpreter','Latex')
axis([0 time(end) -40 40])
% set(gca,'FontSize',18)
grid on
box on
subplot(2,1,2)
hold on
plot(time,rad2deg(des(:,4)),'--','LineWidth',2)
plot(time,yaw(:,1),'b','LineWidth',1)
hold off
title('Yaw Actual/Desired v Time')
xlabel('$t$ (s)','Interpreter','Latex')
ylabel('$Yaw (degreees)$','Interpreter','Latex')
legend('$Yaw$ desired','$Yaw$ Actual','Interpreter','Latex')
axis([0 time(end) -40 40])
% set(gca,'FontSize',18)
grid on
box on

%% xyzpsi states split version for presentation 
% figure
% colororder(colors)
% subplot(1,2,1)
% plot(time,des(:,9),'--','LineWidth',2)
% hold on
% plot(time,states(:,7),'LineWidth',1)
% %title('X Pos Actual/Desired v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$x$ (m)','Interpreter','Latex')
% legend('$x$ desired','$x$ actual','Interpreter','Latex')
% set(gca,'FontSize',18)
% grid on
% subplot(1,2,2)
% plot(time,des(:,4),'--','LineWidth',2)
% hold on
% plot(time,states(:,8),'LineWidth',1)
% %title('Y Pos Actual/Desired v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$y$ (m)','Interpreter','Latex')
% legend('$y$ desired','$y$ actual','Interpreter','Latex')
% set(gca,'FontSize',18)
% grid on
% 
% figure
% colororder(colors)
% subplot(1,2,1)
% plot(time,des(:,1),'--','LineWidth',2)
% hold on
% plot(time,states(:,9),'LineWidth',1)
% %title('Z Pos Actual/Desired v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$z$ (m)','Interpreter','Latex')
% legend('$z$ desired','$z$ actual','Interpreter','Latex')
% set(gca,'FontSize',18)
% grid on
% subplot(1,2,2)
% plot(time,des(:,14),'--','LineWidth',2)
% hold on
% plot(time,states(:,12),'LineWidth',1)
% %title('Psi Actual/Desired v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\psi$ (rad)','Interpreter','Latex')
% legend('$\psi$ desired','$\psi$ actual','Interpreter','Latex')
% set(gca,'FontSize',18)
% grid on

% %% error plots
% 
% figure(3)
% subplot(2,2,1)
% plot(time,s_v1)
% hold on
% plot(time,e_vec_v1(:,1))
% plot(time,e_vec_v1(:,2))
% title('v1 Errors v Time')
% xlabel('time')
% ylabel('error')
% legend('s','e','ed')
% grid on
% subplot(2,2,2)
% plot(time,s_v2)
% hold on
% plot(time,e_vec_v2(:,1))
% plot(time,e_vec_v2(:,2))
% plot(time,e_vec_v2(:,3))
% plot(time,e_vec_v2(:,4))
% title('v2 Errors v Time')
% xlabel('time')
% ylabel('error')
% legend('s','e','ed','edd','eddd')
% grid on
% subplot(2,2,3)
% plot(time,s_v3)
% hold on
% plot(time,e_vec_v3(:,1))
% plot(time,e_vec_v3(:,2))
% plot(time,e_vec_v3(:,3))
% plot(time,e_vec_v3(:,4))
% title('v3 Errors v Time')
% xlabel('time')
% ylabel('error')
% legend('s','e','ed','edd','eddd')
% grid on
% subplot(2,2,4)
% plot(time,s_u4)
% hold on
% plot(time,e_vec_u4(:,1))
% plot(time,e_vec_u4(:,2))
% title('u4 Errors v Time')
% xlabel('time')
% ylabel('error')
% legend('s','e','ed')
% grid on
% 
% %% error v error dot plots
% 
% figure(4)
% subplot(2,2,1)
% plot(e_vec_v1(:,1),e_vec_v1(:,2))
% title('v1 Error Dot v Error')
% xlabel('error')
% ylabel('error dot')
% grid on
% subplot(2,2,2)
% plot(e_vec_v2(:,1),e_vec_v2(:,2))
% title('v2 Error Dot v Error')
% xlabel('error')
% ylabel('error dot')
% grid on
% subplot(2,2,3)
% plot(e_vec_v3(:,1),e_vec_v3(:,2))
% title('v3 Error Dot v Error')
% xlabel('error')
% ylabel('error dot')
% grid on
% subplot(2,2,4)
% plot(e_vec_u4(:,1),e_vec_u4(:,2))
% title('v4 Error Dot v Error')
% xlabel('error')
% ylabel('error dot')
% grid on

%% Voltage Command Signal
V_cmd = U;

figure(3); clf(3);
hold on
plot(time,V_cmd(:,1),'b','LineWidth',1)
plot(time,V_cmd(:,2),'r','LineWidth',1)
hold off
title('Voltage v Time')
xlabel('$t$ (s)','Interpreter','Latex')
ylabel('$V$ (Volts)','Interpreter','Latex')
legend('Rotor Front','Rotor Rear','Interpreter','Latex')
% axis([0 time(end) 631.35 631.5])
% axis([140 time(end) 814 816])
% set(gca,'FontSize',24)
grid on

%% Rotor Speed
rotor(:,1) = Omega(1,1,:);
rotor(:,2) = Omega(2,1,:);
figure(4); clf(4);
hold on
plot(time,rotor(:,1),'b','LineWidth',1)
plot(time,rotor(:,2),'r','LineWidth',1)
hold off
title('Rotor Speed v Time')
xlabel('$t$ (s)','Interpreter','Latex')
ylabel('$\omega$ (rad/s)','Interpreter','Latex')
legend('Rotor Front','Rotor Rear','Interpreter','Latex')
% axis([0 time(end) 631.35 631.5])
% axis([140 time(end) 814 816])
% set(gca,'FontSize',24)
grid on


% %% b/a values settling plots

% figure(6)
% subplot(2,2,1)
% plot(time,haa_hat_v1(:,1))
% hold on
% plot(time,haa_hat_v1(:,2))
% plot(time,haa_hat_v1(:,3))
% title('v1 b/a values v Time')
% xlabel('time')
% ylabel('value')
% legend('h','a1','a2')
% axis([0 2 -70000 160000])
% grid on
% subplot(2,2,2)
% plot(time,haa_hat_v2(:,1))
% hold on
% plot(time,haa_hat_v2(:,2))
% plot(time,haa_hat_v2(:,3))
% plot(time,haa_hat_v2(:,4))
% plot(time,haa_hat_v2(:,5))
% title('v2 h/a values v Time')
% xlabel('time')
% ylabel('value')
% legend('b','a1','a2','a3','a4')
% axis([0 4 0 120])
% grid on
% subplot(2,2,3)
% plot(time,haa_hat_v3(:,1))
% hold on
% plot(time,haa_hat_v3(:,2))
% plot(time,haa_hat_v3(:,3))
% plot(time,haa_hat_v3(:,4))
% plot(time,haa_hat_v3(:,5))
% title('v3 h/a values v Time')
% xlabel('time')
% ylabel('value')
% legend('b','a1','a2','a3','a4')
% axis([0 4 0 120])
% grid on
% subplot(2,2,4)
% plot(time,haa_hat_u4(:,1))
% hold on
% plot(time,haa_hat_u4(:,2))
% plot(time,haa_hat_u4(:,3))
% title('u4 b/a values v Time')
% xlabel('time')
% ylabel('value')
% legend('b','a1','a2')
% axis([0 6 0 500])
% grid on

% %% plot all the states
% figure(7)
% subplot(3,1,1)
% plot(time,states(:,1))
% title('X Velocity v Time')
% xlabel('time')
% ylabel('v (m/s)')
% grid on
% subplot(3,1,2)
% plot(time,states(:,2))
% title('Y Velocity v Time')
% xlabel('time')
% ylabel('v (m/s)')
% grid on
% subplot(3,1,3)
% plot(time,states(:,3))
% title('Z Velocity v Time')
% xlabel('time')
% ylabel('v (m/s)')
% grid on
% 
% figure(8)
% subplot(3,1,1)
% plot(time,states(:,4))
% title('X Angular Velocity v Time')
% xlabel('time')
% ylabel('{\omega} (rad/s)')
% grid on
% subplot(3,1,2)
% plot(time,states(:,5))
% title('Y Angular Velocity v Time')
% xlabel('time')
% ylabel('{\omega} (rad/s)')
% grid on
% subplot(3,1,3)
% plot(time,states(:,6))
% title('Z Angular Velocity v Time')
% xlabel('time')
% ylabel('{\omega} (rad/s)')
% grid on
% 
% figure(9)
% subplot(3,1,1)
% plot(time,states(:,7))
% title('X Position v Time')
% xlabel('time')
% ylabel('position (m)')
% grid on
% subplot(3,1,2)
% plot(time,states(:,8))
% title('Y Position v Time')
% xlabel('time')
% ylabel('position (m)')
% grid on
% subplot(3,1,3)
% plot(time,states(:,9))
% title('Z Position v Time')
% xlabel('time')
% ylabel('position (m)')
% grid on
% 
% figure(10)
% subplot(3,1,1)
% plot(time,states(:,10))
% title('Roll Angle v Time')
% xlabel('time')
% ylabel('{\phi} (rad)')
% grid on
% subplot(3,1,2)
% plot(time,states(:,11))
% title('Pitch Angle v Time')
% xlabel('time')
% ylabel('{\theta} (rad)')
% grid on
% subplot(3,1,3)
% plot(time,states(:,12))
% title('Yaw Angle v Time')
% xlabel('time')
% ylabel('{\psi} (rad)')
% grid on

%% h values nominal vs sim
% ts = [0;time(end)];
% hx = [1085.7;1085.7];
% hy = [1085.7;1085.7];
% hz = [1.625714286*10^5;1.625714286*10^5];
% hpsi = [48846.15386;48846.15386];
% ax1 = [-9185.714286;-9185.714286];
% ax2 = [8900.000000;8900.000000];
% ax3 = [-1085.714286;-1085.714286];
% ay1 = [-9185.714286;-9185.714286];
% ay2 = [8900.000000;8900.000000];
% ay3 = [-1085.714286;-1085.714286];
% az1 = [-1.625714286*10^5;-1.625714286*10^5];
% apsi1 = [7692.307696,7692.307696];
% apsi2 = [-48846.15386;-48846.15386];

% figure
% colororder(colors)
% subplot(2,2,1)
% plot(ts,hx,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v3(:,1),'LineWidth',1)
% %title('v1 h hat/h v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h_x$ (m)','Interpreter','Latex')
% legend('$h_x$','$\hat{h}_x$','Interpreter','Latex','Location','southeast')
% % axis([0 time(end) -20 1200])
% % axis([0 50 0 4000])
% set(gca,'FontSize',18)
% grid on
% subplot(2,2,2)
% plot(ts,hy,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v2(:,1),'LineWidth',1)
% %title('v2 b hat/b v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h_y$ (m)','Interpreter','Latex')
% legend('$h_y$','$\hat{h}_y$','Interpreter','Latex','Location','southeast')
% % axis([0 time(end) -20 1200])
% % axis([0 50 0 4000])
% set(gca,'FontSize',18)
% grid on
% subplot(2,2,3)
% plot(ts,hz,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v1(:,1),'LineWidth',1)
% %title('v3 b hat/b v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h_z$ (m$^{-1}$)','Interpreter','Latex')
% legend('$h_z$','$\hat{h}_z$','Interpreter','Latex','Location','southeast')
% % axis([0 time(end) -50000 200000])
% % axis([0 300 150000 170000])
% set(gca,'FontSize',18)
% grid on
% subplot(2,2,4)
% plot(ts,hpsi,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_u4(:,1),'LineWidth',1)
% %title('v4 b hat/b v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h_{\psi}$ (non-dimensional)','Interpreter','Latex')
% legend('$h_{\psi}$','$\hat{h}_{\psi}$','Interpreter','Latex','Location','southeast')
% % axis([0 time(end) -100 50000])
% % axis([0 3 48000 52000])
% set(gca,'FontSize',18)
% grid on

%% a values nominal vs sim


% figure
% colororder(colors)
% subplot(1,3,1)
% plot(ts,ax1,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v3(:,2),'LineWidth',1)
% %title('a1x hat/a1x nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{x1}$ (m)','Interpreter','Latex')
% legend('$a_{x1}$','$\hat{a}_{x1}$','Interpreter','Latex','Location','southeast')
% % axis([0 4 700 760])%-1 1.5
% set(gca,'FontSize',18)
% grid on
% subplot(1,3,2)
% plot(ts,ax2,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v3(:,3),'LineWidth',1)
% %title('a2x hat/a2x nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{x2}$ (m)','Interpreter','Latex')
% legend('$a_{x2}$','$\hat{a}_{x2}$','Interpreter','Latex')
% % axis([0 4 -760 -700])% 0 1.5
% set(gca,'FontSize',18)
% grid on
% subplot(1,3,3)
% plot(ts,ax3,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v3(:,4),'LineWidth',1)
% %title('a3x hat/a3x nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{x3}$ (m)','Interpreter','Latex')
% legend('$a_{x3}$','$\hat{a}_{x3}$','Interpreter','Latex')
% % axis([0 4 -2000 -600]);%0.5 1.5
% set(gca,'FontSize',18)
% grid on

% figure
% colororder(colors)
% subplot(1,3,1)
% plot(ts,ay1,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v2(:,2),'LineWidth',1)
% %title('a1y hat/a1y nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{y1}$ (m)','Interpreter','Latex')
% legend('$a_{y1}$','$\hat{a}_{y1}$','Interpreter','Latex','Location','southeast')
% % axis([0 4 700 760])%-1 1.5
% set(gca,'FontSize',18)
% grid on
% subplot(1,3,2)
% plot(ts,ay2,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v2(:,3),'LineWidth',1)
% %title('a2y hat/a2y nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{y2}$ (m)','Interpreter','Latex')
% legend('$a_{y2}$','$\hat{a}_{y2}$','Interpreter','Latex','Location','southeast')
% % axis([0 4 -760 -700])%0 1.5
% set(gca,'FontSize',18)
% grid on
% subplot(1,3,3)
% plot(ts,ay3,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v2(:,4),'LineWidth',1)
% %title('a3y hat/a3y nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{y3}$ (m)','Interpreter','Latex')
% legend('$a_{y3}$','$\hat{a}_{y3}$','Interpreter','Latex','Location','southeast')
% % axis([0 4 -2000 -600])%.5 1.5
% set(gca,'FontSize',18)
% grid on

% figure
% colororder(colors)
% plot(ts,az1,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_v1(:,2),'LineWidth',1)
% %title('a1z hat/a1z nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{z1}$ (m$^{-1}$)','Interpreter','Latex')
% legend('$a_{z1}$','$\hat{a}_{z1}$','Interpreter','Latex')
% % axis([0 2 -70000 10000])
% set(gca,'FontSize',18)
% grid on

% figure
% colororder(colors)
% subplot(1,2,1)
% plot(ts,apsi1,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_u4(:,2),'LineWidth',1)
% %title('a1{\psi} hat/a1{\psi} nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{\psi 1}$ (non-dimensional)','Interpreter','Latex')
% legend('$a_{\psi 1}$','$\hat{a}_{\psi 1}$','Interpreter','Latex')
% % axis([0 4 700 900])%-.25 1.25
% set(gca,'FontSize',18)
% grid on
% subplot(1,2,2)
% plot(ts,apsi2,'--','LineWidth',2)
% hold on
% plot(time,haa_hat_u4(:,3),'LineWidth',1)
% %title('a1{\psi} hat/a1{\psi} nominal v Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_{\psi 2}$ (non-dimensional)','Interpreter','Latex')
% legend('$a_{\psi 2}$','$\hat{a}_{\psi 2}$','Interpreter','Latex')
% % axis([0 4 -60000 -40000])%-.25 1.25
% set(gca,'FontSize',18)
% grid on


%% Z error plot 

% m = 1.138;
% ct = 7e-6;
% g = 9.81;
% zh = m/ct;
% za1 = -m/ct;
% zf1 = -g;
% zlambda = 3;
% zphis = 0.05;
% zz = -0.35:0.01:0.15;
% zerrorline = -zlambda*zz;
% zblu = zerrorline+zphis; %boundary layer upper
% zbll = zerrorline-zphis; %boundary layer lower
% zref = des(:,1) - (zlambda * e_vec_v1(:,2));
% zRHS = (haa_hat_v1(:,1)-zh).*zref + (haa_hat_v1(:,2)-za1)*(zf1);

% figure(5)
% subplot(2,2,3)
% plot(time,zRHS,'b','LineWidth',1)
% hold on
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\rho_z$','Interpreter','Latex')
% legend('(NASMC)','Interpreter','Latex')
% grid on
% set(gca,'FontSize',18)

% figure
% plot(e_vec_v1(:,1),e_vec_v1(:,2),'LineWidth',1)
% hold on
% plot(zz,zerrorline,'--k')
% plot(zz,zblu,'--r')
% plot(zz,zbll,'--r')
% xlabel('$e_z$','Interpreter','Latex')
% ylabel('$\dot{e}_z$','Interpreter','Latex')
% grid on
% axis equal
% set(gca,'FontSize',18)

%% X error plot

xlambda = 2;
xphis = 0.05;
xx = -0.1:0.01:0.12;
xerrorline = -xlambda*xx;
xblu = xerrorline+xphis; %boundary layer upper
xbll = xerrorline-xphis; %boundary layer lower
% Ix = 0.0077;
% Iy = 0.0075;
% Ixy = 0.0076;
% Iz = 0.0127;
% xh = Ixy/ct;
% xa1 = -(Ix-Iz)/ct;
% xa2 = -(Iz-Iy)/ct;
% xa3 = -Ixy/ct;
% xf1 = cos(states(:,11)).*cos(states(:,12)).*states(:,4).*states(:,6).*f_hats_over_m(:,1);
% xf2 = -cos(states(:,11)).*sin(states(:,12)).*states(:,5).*states(:,6).*f_hats_over_m(:,1);
% xf3 = cos(states(:,11)).*cos(states(:,12)).*(2*states(:,5).*f_hats_over_m(:,2)+states(:,4).*states(:,6).*f_hats_over_m(:,1))...
%       -cos(states(:,11)).*cos(states(:,12)).*(-2*states(:,4).*f_hats_over_m(:,2)+states(:,5).*states(:,6).*f_hats_over_m(:,1))...
%       + sin(states(:,11)).*(f_hats_over_m(:,3)+(-states(:,4).^2 - states(:,5).^2).*f_hats_over_m(:,1));
% xref = des(:,9) - 3*xlambda*e_vec_v3(:,4) - 3*xlambda^2*e_vec_v3(:,3) - xlambda^3*e_vec_v3(:,2);
% xRHS = (haa_hat_v3(:,1)-xh).*xref + (haa_hat_v3(:,2)-xa1).*xf1 + (haa_hat_v3(:,3)-xa2).*xf2 + (haa_hat_v3(:,4)-xa3).*xf3;

% figure(6)
% subplot(2,2,1)
% plot(time,xRHS,'b','LineWidth',1)
% hold on
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\rho_x$','Interpreter','Latex')
% legend('(NASMC)','Interpreter','Latex')
% grid on
% set(gca,'FontSize',18)

% figure
% plot(e_vec_v3(:,1),e_vec_v3(:,2),'LineWidth',1)
% hold on
% plot(xx,xerrorline,'--k')
% plot(xx,xblu,'--r')
% plot(xx,xbll,'--r')
% xlabel('$e_x$','Interpreter','Latex')
% ylabel('$\dot{e}_x$','Interpreter','Latex')
% grid on
% axis equal
% set(gca,'FontSize',18)

%% Y error plot

% ylambda = 2;
% yphis = 0.05;
% yy = -0.1:0.01:0.12;
% yerrorline = -ylambda*yy;
% yblu = yerrorline+yphis; %boundary layer upper
% ybll = yerrorline-yphis; %boundary layer lower
% yh = Ixy/ct;
% ya1 = -(Ix-Iz)/ct;
% ya2 = -(Iz-Iy)/ct;
% ya3 = -Ixy/ct;
% yf1 = -(cos(states(:,4)).*sin(states(:,6))+sin(states(:,4)).*sin(states(:,5)).*cos(states(:,6))).*states(:,4).*states(:,6).*f_hats_over_m(:,1);
% yf2 = (cos(states(:,4)).*cos(states(:,6))-sin(states(:,4)).*sin(states(:,5)).*sin(states(:,6))).*states(:,5).*states(:,6).*f_hats_over_m(:,1);
% yf3 = (cos(states(:,4)).*sin(states(:,6))+sin(states(:,4)).*sin(states(:,5)).*cos(states(:,6))).*(2*states(:,5).*f_hats_over_m(:,2)+states(:,4).*states(:,6).*f_hats_over_m(:,1))...
%       +(cos(states(:,4)).*cos(states(:,6))-sin(states(:,4)).*sin(states(:,5)).*sin(states(:,6))).*(-2*states(:,4).*f_hats_over_m(:,2)+states(:,5).*states(:,6).*f_hats_over_m(:,1))...
%       - sin(states(:,4)).*cos(states(:,5)).*(f_hats_over_m(:,3)+(-states(:,4).^2 - states(:,5).^2).*f_hats_over_m(:,1));
% yref = des(:,4) - 3*ylambda*e_vec_v2(:,4) - 3*ylambda^2*e_vec_v2(:,3) - ylambda^3*e_vec_v2(:,2);
% yRHS = (haa_hat_v2(:,1)-yh).*yref + (haa_hat_v2(:,2)-ya1).*yf1 + (haa_hat_v2(:,3)-ya2).*yf2 + (haa_hat_v2(:,4)-ya3).*yf3;

% figure(7)
% subplot(2,2,2)
% plot(time,yRHS,'b','LineWidth',1)
% hold on
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\rho_y$','Interpreter','Latex')
% legend('(NASMC)','Interpreter','Latex')
% grid on
% set(gca,'FontSize',18)

% figure
% plot(e_vec_v2(:,1),e_vec_v2(:,2),'LineWidth',1)
% hold on
% plot(yy,yerrorline,'--k')
% plot(yy,yblu,'--r')
% plot(yy,ybll,'--r')
% xlabel('$e_y$','Interpreter','Latex')
% ylabel('$\dot{e}_y$','Interpreter','Latex')
% grid on
% axis equal
% set(gca,'FontSize',18)

%% Psi error plot
% cq = 2.6e-7;
% ph = Iz/cq;
% pa1 = -(Iy-Ix)/cq;
% pa2 = -Iz/cq;
% pf1 = -states(:,4).*states(:,5);
% pf2a = ((sin(states(:,6)).*sin(states(:,5)).*cos(states(:,5)).*statesd(:,6) - cos(states(:,6)).*statesd(:,5))./((cos(states(:,5))).^2)).*states(:,4);
% pf2b = ((cos(states(:,6)).*sin(states(:,5)).*cos(states(:,5)).*statesd(:,6) + sin(states(:,6)).*statesd(:,5))./((cos(states(:,5))).^2)).*states(:,5);
% pf2c = (-cos(states(:,6)).*sin(states(:,5))./cos(states(:,5))).*statesd(:,4);
% pf2d = (sin(states(:,6)).*sin(states(:,5))./cos(states(:,5))).*statesd(:,5);
% pf2 = pf2a + pf2b + pf2c + pf2d;
plambda = 1;
pphis = 0.05;
pp = -0.1:0.01:0.8;
perrorline = -plambda*pp;
pblu = perrorline+pphis; %boundary layer upper
pbll = perrorline-pphis; %boundary layer lower
% pref = des(:,14) - (plambda .* e_vec_u4(:,2));
% pRHS = (haa_hat_u4(:,1)-ph).*pref + (haa_hat_u4(:,2)-pa1).*(pf1) + (haa_hat_u4(:,3)-pa2).*(pf2);

% figure(8)
% subplot(2,2,4)
% plot(time,pRHS,'b','LineWidth',1)
% hold on
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\rho_\psi$','Interpreter','Latex')
% legend('(NASMC)','Interpreter','Latex')
% grid on
% set(gca,'FontSize',18)

% figure
% plot(e_vec_u4(:,1),e_vec_u4(:,2),'LineWidth',1)
% hold on
% plot(pp,perrorline,'--k')
% plot(pp,pblu,'--r')
% plot(pp,pbll,'--r')
% xlabel('$e_{\psi}$','Interpreter','Latex')
% ylabel('$\dot{e}_{\psi}$','Interpreter','Latex')
% grid on
% axis equal
% set(gca,'FontSize',18)