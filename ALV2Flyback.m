function [t,r,gamma,v,m,xi,phi,zeta] = ALV2Flyback(t0,r0,gamma0,v0,xi0,phi0,zeta0)
% Flyback Simulation of the ALV-2 First Stages
% Sholto Forbes-Spyratos

% Set Conditions
r_E = 6371000; % radius of Earth (m)

dt = 0.1; % Timestep (s)

% Initialise Position Arrays
t(1) = t0;
r(1) = r0;
gamma(1) = gamma0;
v(1) = v0;
xi(1) = xi0;
phi(1) = phi0;
zeta(1) = zeta0;

m = 480; % Mass (kg)
alpha = deg2rad(30);
A = 0.5; % Reference Area (m^2)

i = 1;

T = 0; % Thrust

while v > 500
% Calculate Aerodynamics
atmosphere = dlmread('atmosphere.txt');

rho = interp1(atmosphere(:,1),atmosphere(:,4),r(1)-r_E);

Cl = 1.1*sin(alpha)^2*cos(alpha);
Cd = 1.1*sin(alpha)^3 + 0.01;

L(i) = 0.5* Cl * v(i)^2 * A * rho;
D(i) = 0.5* Cd * v(i)^2 * A * rho;
    
% Increment Equations of Motion
[rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L(i),D(i),T,m,alpha);

t(i+1) = t(i) + dt;
r(i+1) = r(i) + dt*rdot;
xi(i+1) = xi(i) + dt*xidot;
phi(i+1) = phi(i) + dt*phidot;
gamma(i+1) = gamma(i) + dt*gammadot;
v(i+1) = v(i) + dt*vdot;
zeta(i+1) = zeta(i) + dt*zetadot;


i = i+1;
end


end

