function [rdiff] = ALV2FUNCTION(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Atmosphere Data (1976 NASA Model)
atmosphere = dlmread('atmosphere.txt');

% AeroCoefficients
% PLACEHOLDERS, DO NOT CHANGE BETWEEN STAGES

AeroCoeffs = dlmread('AeroCoeffs.txt');
AeroCoeffs = [[0 0];AeroCoeffs];

% Thrust Libraries
% Altitude(km)  Pressure(atm)  Effective Exhaust Velocity(m/s)
% Specific impulse(s)  Thrust Coefficient  Thrust(kN)
FirstStageThrust = dlmread('FirstStageThrust.txt');
SecondStageThrust = dlmread('SecondStageThrust.txt');
ThirdStageThrust = dlmread('ThirdStageThrust.txt');

% no. First Stage Modules
N = 4;

A1 = .28 + N*0.5; % Reference Area of first stage with 4 boosters, each booster is 0.5 and core stage is 0.28 (m^2)
A2 = 0.28;

% Define First Stage Characteristics For 1 Module

mP1 = 1600; % First Stage Propellant Loading (Total) (kg)
PCR1 = 16.39; % First Stage Propellant Consumption Rate (Total) (kg/s)
mB1 = 480; % First Stage Burnout Mass (kg)
mAF = 70; % Aero Fuel (kg)
mL = 410; % Landing Mass (kg)

% Define Launch System Characteristics Which are Dependent on no. of First Stage Modules
if N == 4
    mPayload = 18; % Payload Mass (kg)
    mCF = 7; % Container and Fairing Mass (kg)
    mP2 = 930; % Second Stage Propellant Loading (Total) (kg)
    PCR2 = 3.952; % Second Stage Propellant Consumption Rate (Total) (kg/s)
    mB2 = 228; % Second Stage Burnout Mass (kg)
    mP3 = 145; % Third Stage Propellant Loading (Total) (kg)
    PCR3 = 0.4744; % Third Stage Propellant Consumption Rate (Total) (kg/s)
    mB3 = 40; % Third Stage Burnout Mass (kg)
elseif N == 3
    mPayload = 14.5;
    mCF = 5.8;
    mP2 = 700;
    PCR2 = 2.964;
    mB2 = 171;
    mP3 = 109;
    PCR3 = 0.2372;
    mB3 = 30;
elseif N == 2
    mPayload = 7.5;
    mCF = 4.5;
    mP2 = 455;
    PCR2 = 1.976;
    mB2 = 112;
    mP3 = 71;
    PCR3 = 0.2372;
    mB3 = 20;
elseif N == 1
    mPayload = 2;
    mCF = 3;
    mP2 = 235;
    PCR2 = 0.988;
    mB2 = 57;
    mP3 = 38;
    PCR3 = 0.2372;
    mB3 = 12;
end

% Set Conditions
r_E = 6371000; % radius of Earth (m)

dt = 0.1; % Timestep (s)



%==========================================================================
%----------------------- First Stage Simulation ---------------------------
%==========================================================================

% Cartesian Coordinates

L = 0; % Lift, none
alpha = 0; % Angle of Attack, zero

% Initialise Arrays
r(1) = r_E;
xi(1) = 0;
phi(1) = 0;
gamma(1) = deg2rad(90);
v(1) = 0;
zeta(1) = 0;

m(1) = (mP1 + mAF + mL)*N  + mPayload + mCF + mP2 + mB2+ mP3 + mB3; % Mass Array (kg)
t(1) = 0; % Time Array (s)
v_a = interp1(atmosphere(:,1),atmosphere(:,5),r(1)-r_E); % Speed of Sound
M(1) = v(1)/v_a; % Mach no Array
Cd(1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(1)); % Drag Coefficient Array (N);
D(1) = 0.5* Cd * v(1)^2 * A1 * interp1(atmosphere(:,1),atmosphere(:,4),r(1)-r_E);
T(1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),0)*N*3  * 1000; % Thrust Array (kN) (3 Engines on Each ALV-2 First Stage Module)

mParray(1) = mP1*N; % Initialise Propellant Mass Array (kg)


i = 1; % Temporary Iteration Counter

pitchover = 'no';

t_pitch = 20; % pitchover time

while mParray(i) > 0
    
% Increment Equations of Motion
[rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L,D(i),T(i),m(i),alpha);

r(i+1) = r(i) + dt*rdot;
xi(i+1) = xi(i) + dt*xidot;
phi(i+1) = phi(i) + dt*phidot;
gamma(i+1) = gamma(i) + dt*gammadot;
v(i+1) = v(i) + dt*vdot;
zeta(i+1) = zeta(i) + dt*zetadot;
    
% Increment Vehicle Parameters
    
mParray(i+1) = mParray(i) - PCR1*N*dt; 

m(i+1) = m(i) - PCR1*N*dt;

if r(i+1)-r_E < 48000
T(i+1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),(r(i+1)-r_E)/1000) * N*3 * 1000; 
else
T(i+1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),48000/1000) * N*3  * 1000;
end

if r(i+1)-r_E < 85000
v_a = interp1(atmosphere(:,1),atmosphere(:,5),r(i+1)-r_E);
else
v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
end

M(i+1) = v(i+1)/v_a;

Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 

if r(i+1)-r_E < 85000
D(i+1) = 0.5 * Cd(i+1) * v(i+1)^2 * A1 * interp1(atmosphere(:,1),atmosphere(:,4),r(i+1)-r_E);
else
D(i+1) = 0;
end

% Initiate Turn, prevent gamma from changing before pitchover time
if t(i) < t_pitch
gamma(i+1) = deg2rad(90);
zeta(i+1) = zeta(i);
elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 0 % Pitchover Time (Arbitrary Currently)
gamma(i+1) = deg2rad(89); % Pitchover Angle of 1 Degree (assumed to be instantaneous)
zeta(i+1) = zeta(i);
pitchover = 'yes';

alpha = deg2rad(-1); % set angle of attack after turn start

end

t(i+1) = t(i) + dt;

i = i+1;
end
temp_1 = i;






%==========================================================================
%----------------------- Second Stage Simulation --------------------------
%==========================================================================

alpha = deg2rad(-5); % set angle of attack


m(i) = m(i) - mB1*N;

mParray(i) = mP2;


while mParray(i) > 0
    
% Increment Equations of Motion
[rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L,D(i),T(i),m(i),alpha);

r(i+1) = r(i) + dt*rdot;
xi(i+1) = xi(i) + dt*xidot;
phi(i+1) = phi(i) + dt*phidot;
gamma(i+1) = gamma(i) + dt*gammadot;
v(i+1) = v(i) + dt*vdot;
zeta(i+1) = zeta(i) + dt*zetadot;
    
% Increment Vehicle Parameters
    
mParray(i+1) = mParray(i) - PCR2*dt; 

m(i+1) = m(i) - PCR2*dt;

if r(i+1)-r_E < 48000
T(i+1) = interp1(SecondStageThrust(:,1),SecondStageThrust(:,6),(r(i+1)-r_E)/1000) * N  * 1000; 
else
T(i+1) = interp1(SecondStageThrust(:,1),SecondStageThrust(:,6),48000/1000) * N  * 1000;
end

if r(i+1)-r_E < 85000
v_a = interp1(atmosphere(:,1),atmosphere(:,5),r(i+1)-r_E);
else
v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
end

M(i+1) = v(i+1)/v_a;

Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 

if r(i+1)-r_E < 85000
D(i+1) = 0.5 * Cd(i+1) * v(i+1)^2 * A2 * interp1(atmosphere(:,1),atmosphere(:,4),r(i+1)-r_E);
else
D(i+1) = 0;
end

t(i+1) = t(i) + dt;

i = i+1;
end
temp_1 = i;


%==========================================================================
%----------------------- Third Stage Simulation --------------------------
%==========================================================================


m(i) = m(i) - mB2;

mParray(i) = mP3;


j = 1;
t_temp(1) = 0; % initiate temporary time scale for alpha calculation

while mParray(i) > 0
    

alpha = deg2rad( x(1)*t_temp(j) + x(2)); %determine angle of attack

j = j+1;
t_temp(j) = t_temp(j-1) + dt;


    
% Increment Equations of Motion
[rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L,D(i),T(i),m(i),alpha);

r(i+1) = r(i) + dt*rdot;
xi(i+1) = xi(i) + dt*xidot;
phi(i+1) = phi(i) + dt*phidot;
gamma(i+1) = gamma(i) + dt*gammadot;
v(i+1) = v(i) + dt*vdot;
zeta(i+1) = zeta(i) + dt*zetadot;
    
% Increment Vehicle Parameters
    
mParray(i+1) = mParray(i) - PCR3*dt; 

m(i+1) = m(i) - PCR3*dt;

if r(i+1)-r_E < 48000
T(i+1) = interp1(ThirdStageThrust(:,1),ThirdStageThrust(:,6),(r(i+1)-r_E)/1000) * N  * 1000; 
else
T(i+1) = interp1(ThirdStageThrust(:,1),ThirdStageThrust(:,6),48000/1000) * N  * 1000;
end

D(i+1) = 0;


t(i+1) = t(i) + dt;

i = i+1;
end
temp_1 = i;


% absendgamma = abs(gamma(end))

% rdiff = abs((r(end)-r_E)-400000)


rdiff = abs((r(end)-r_E)-400000) + 10*abs(gamma(end))

end

