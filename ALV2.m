% ALV-2 Trajectory Simulation
% Sholto Forbes-Spyratos
clear all

h = msgbox('ALV-2 Trajectory Simulation is Running');


% Initial Conditions
% r0 = 0; % Altitude (m)
% xi0 = deg2rad(153); % Longitude (rad)
% phi0 = deg2rad(-27); % Latitude (rad)
% gamma0 = deg2rad(90); % Flight Path Angle (rad)
% zeta0 = deg2rad(97); % Heading Angle (rad)

prompt = {'Launch Altitude (km)','Launch Longitude (deg)','Launch Latitude (deg)', 'Launch Angle (deg)', 'Launch Heading Angle (deg)', 'Target Altitude (km)'};
dlg_title = 'Inputs';
num_lines = 1;
defaultans = {'0','153','-27','90','97', '400'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);

% Initial Conditions
r0 = str2num(answer{1})*1000; % Altitude (m)
xi0 = deg2rad(str2num(answer{2})); % Longitude (rad)
phi0 = deg2rad(str2num(answer{3})); % Latitude (rad)
gamma0 = deg2rad(str2num(answer{4})); % Flight Path Angle (rad)
zeta0 = deg2rad(str2num(answer{5})); % Heading Angle (rad)
rTarget = str2num(answer{6})*1000; % Target Altitude (m)

% optimisation parameters
A = [];
b = [];
Aeq = [];
beq = [];

% Angle of attack is a linear function of time At + B

% Nodes = 20; % No. Nodes Evaluated, For Each Stage

% lb = [-10,-10,-10,-10]; % lower bounds of A, B
% ub = [10,10,10,10]; % upper bounds of A, B
% 
% x0 = [0,0,0,0];

% lb = -10*ones(1,Nodes*2); % lower bounds of A, B
% ub = 10*ones(1,Nodes*2); % upper bounds of A, B

% lb = -.1*ones(1,Nodes*2); % lower bounds of A, B
% ub = .1*ones(1,Nodes*2); % upper bounds of A, B
% 
% x0 = zeros(1,Nodes*2);

lb = [0,deg2rad(70)]; % lower bounds of A, B
ub = [1,deg2rad(90)]; % upper bounds of A, B

x0 = [0.01,deg2rad(80)];

nonlcon = [];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','UseParallel',true);
% 
x = fmincon(@(x)ALV2FUNCTION(x,r0,gamma0,xi0,phi0,zeta0,rTarget),x0,A,b,Aeq,beq,lb,ub,nonlcon, options)  % run trajectory calculation routine, this determines the correct angle of attack schedule for the upper stages


[diff,t,r,gamma,v,m,xi,phi,zeta,i12,i23,alpha] = ALV2FUNCTION(x,r0,gamma0,xi0,phi0,zeta0,rTarget); % simulate trajectory

r_E = 6371000; % radius of Earth (m)

figure(1)

subplot(5,1,1)
hold on
plot(t(1:i12),(r(1:i12)-r_E)/1000,'LineWidth',1.5,'Color','b')
plot(t(i12:i23),(r(i12:i23)-r_E)/1000,'LineWidth',1.5,'Color','r')
plot(t(i23:end),(r(i23:end)-r_E)/1000,'LineWidth',1.5,'Color','g')
ylabel('Altitude (km)');
subplot(5,1,2)
hold on
plot(t(1:i12),rad2deg(gamma(1:i12)),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),rad2deg(gamma(i12:i23)),'LineWidth',1.5,'Color','r')
plot(t(i23:end),rad2deg(gamma(i23:end)),'LineWidth',1.5,'Color','g')
ylabel('Trajectory Angle (deg)');
subplot(5,1,3)
hold on
plot(t(1:i12),v(1:i12),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),v(i12:i23),'LineWidth',1.5,'Color','r')
plot(t(i23:end),v(i23:end),'LineWidth',1.5,'Color','g')
ylabel('Velocity (m/s)');
subplot(5,1,4)
hold on
plot(t(1:i12),m(1:i12),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),m(i12:i23),'LineWidth',1.5,'Color','r')
plot(t(i23:end),m(i23:end),'LineWidth',1.5,'Color','g')
ylabel('Mass (kg)');
% xlabel('Time (s)');
subplot(5,1,5)
hold on
plot(t(1:i12),rad2deg(alpha(1:i12)),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),rad2deg(alpha(i12:i23)),'LineWidth',1.5,'Color','r')
plot(t(i23:end),rad2deg(alpha(i23:end)),'LineWidth',1.5,'Color','g')
ylabel('Angle of Attack (deg)');
xlabel('Time (s)');

figure(2)
hold on
geoshow('landareas.shp', 'FaceColor', [1.0 1.0 1.0]);
geoshow(rad2deg(phi(1:i12)),rad2deg(xi(1:i12)),'LineWidth',2,'Color','b')
geoshow(rad2deg(phi(i12:i23)),rad2deg(xi(i12:i23)),'LineWidth',2,'Color','r')
geoshow(rad2deg(phi(i23:end)),rad2deg(xi(i23:end)),'LineWidth',2,'Color','g')


%==========================================================================
% ---------------------------    Flyback    -------------------------------
%==========================================================================

% % lb = [-0.1, -45.]; % lower bounds of A, B
% % ub = [0.1,45.]; % upper bounds of A, B
% % 
% % x0 = [0,0];
% 
% % x = fmincon(@(x)ALV2Flyback(x, t(i12),r(i12),gamma(i12),v(i12),xi(i12),phi(i12),zeta(i12)),x0,A,b,Aeq,beq,lb,ub,nonlcon, options)

% [rdiff,t_fb,r_fb,gamma_fb,v_fb,m_fb,xi_fb,phi_fb,zeta_fb] = ALV2Flyback(45, t(i12),r(i12),gamma(i12),v(i12),xi(i12),phi(i12),zeta(i12));
% 
% 
% figure(3)
% subplot(4,1,1)
% plot(t_fb,(r_fb-r_E)/1000,'LineWidth',1.5,'Color','b')
% subplot(4,1,2)
% plot(t_fb,rad2deg(gamma_fb),'LineWidth',1.5,'Color','b')
% subplot(4,1,3)
% plot(t_fb,v_fb,'LineWidth',1.5,'Color','b')











