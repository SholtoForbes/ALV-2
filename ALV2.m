% ALV-2 Trajectory Simulation
% Sholto Forbes-Spyratos
clear all

% optimisation parameters
A = [];
b = [];
Aeq = [];
beq = [];

% Angle of attack is a linear function of time At + B
lb = [-.1,-10]; % lower bounds of A, B
ub = [.1,10]; % upper bounds of A, B

x0 = [0,0];

nonlcon = [];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','UseParallel',true);

x = fmincon(@ALV2FUNCTION,x0,A,b,Aeq,beq,lb,ub,nonlcon, options)  % run trajectory calculation routine, this determines the correct angle of attack schedule for the upper stages


[rdiff,t,r,gamma,v,m,xi,phi,zeta,i12,i23] = ALV2FUNCTION(x); % simulate trajectory

r_E = 6371000; % radius of Earth (m)

figure(1)

subplot(4,1,1)
hold on
plot(t(1:i12),(r(1:i12)-r_E)/1000,'LineWidth',1.5,'Color','b')
plot(t(i12:i23),(r(i12:i23)-r_E)/1000,'LineWidth',1.5,'Color','r')
plot(t(i23:end),(r(i23:end)-r_E)/1000,'LineWidth',1.5,'Color','g')
ylabel('Altitude (km)');
subplot(4,1,2)
hold on
plot(t(1:i12),rad2deg(gamma(1:i12)),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),rad2deg(gamma(i12:i23)),'LineWidth',1.5,'Color','r')
plot(t(i23:end),rad2deg(gamma(i23:end)),'LineWidth',1.5,'Color','g')
ylabel('Trajectory Angle (deg)');
subplot(4,1,3)
hold on
plot(t(1:i12),v(1:i12),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),v(i12:i23),'LineWidth',1.5,'Color','r')
plot(t(i23:end),v(i23:end),'LineWidth',1.5,'Color','g')
ylabel('Velocity (m/s)');
subplot(4,1,4)
hold on
plot(t(1:i12),m(1:i12),'LineWidth',1.5,'Color','b')
plot(t(i12:i23),m(i12:i23),'LineWidth',1.5,'Color','r')
plot(t(i23:end),m(i23:end),'LineWidth',1.5,'Color','g')
ylabel('Mass (kg)');
xlabel('Time (s)');


figure(2)
hold on
geoshow('landareas.shp', 'FaceColor', [1.0 1.0 1.0]);
geoshow(rad2deg(phi(1:i12)),rad2deg(xi(1:i12)),'LineWidth',2,'Color','b')
geoshow(rad2deg(phi(i12:i23)),rad2deg(xi(i12:i23)),'LineWidth',2,'Color','r')
geoshow(rad2deg(phi(i23:end)),rad2deg(xi(i23:end)),'LineWidth',2,'Color','g')


% =========================================================================
% ---------------------------    Flyback    -------------------------------
%==========================================================================



[t_fb,r_fb,gamma_fb,v_fb,m_fb,xi_fb,phi_fb,zeta_fb] = ALV2Flyback(t(i12),r(i12),gamma(i12),v(i12),xi(i12),phi(i12),zeta(i12));


figure(3)
subplot(4,1,1)
plot(t_fb,(r_fb-r_E)/1000,'LineWidth',1.5,'Color','b')
subplot(4,1,2)
plot(t_fb,rad2deg(gamma_fb),'LineWidth',1.5,'Color','b')
subplot(4,1,3)
plot(t_fb,v_fb,'LineWidth',1.5,'Color','b')











