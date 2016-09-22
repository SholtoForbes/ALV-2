function dz = rocketDynamics(z,u,phase)
global SCALE

h = z(1,:);   %Height
v = z(2,:);   %Velocity
m = z(3,:);   %Mass
gamma = z(4,:);
alpha = z(5,:);

% alpha = u(1,:);

dalphadt = u(1,:);

if isnan(gamma)
    gamma = 1.5708;
end

% T = u(1,:);        %Thrust
T = 13000*3*SCALE^2; % THIS SCALING IS JUST FOR THE ENGINE...

density = 1.474085291*(0.9998541833.^h);  %Data fit off of wolfram alpha


A= 0.5*SCALE^2;
speedOfSound = 280;  %(m/s)  %At 10 km altitude
mach = v/speedOfSound;


AeroCoeffs = dlmread('AeroCoeffs.txt');

Cd = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),mach);
D = 0.5*Cd.*A.*density.*v.^2;


%%%% Compute gravity from inverse-square law:
rEarth = 6.3674447e6;  %(m) radius of earth
mEarth = 5.9721986e24;  %(kg) mass of earth
G = 6.67e-11; %(Nm^2/kg^2) gravitational constant
g = G*mEarth./((h+rEarth).^2);

%%%% Complete the calculation:
global Tmax
% dm = -60*ones(1,length(h)).*T/Tmax.*Tmax/200000;   %mass rate
dm = -16.39*ones(1,length(h))*SCALE^2;

% alpha = 0*ones(1,length(h));


xi = 0*ones(1,length(h));
phi = 0*ones(1,length(h));
zeta = 0*ones(1,length(h));
% L = 0*ones(1,length(h));
L = 0.5*Cd/10.*A.*density.*v.^2.*rad2deg(alpha);

switch phase
    case 'prepitch'
    gamma = 1.5708*ones(1,length(h)); % Control Trajectory Angle 
    case 'postpitch'
    %Do nothing
end


[dr,dxi,dphi,dgamma,dv,dzeta] = RotCoords(h+rEarth,xi,phi,gamma,v,zeta,L,D,T,m,alpha,phase);

if isnan(dgamma)
dgamma = 0;
end

dz = [dr;dv;dm;dgamma;dalphadt];

end