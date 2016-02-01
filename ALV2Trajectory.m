% ALV-2 Trajectory Simulation 
% Created by Sholto Forbes 27/1/16
clear all

% Atmosphere Data (1976 NASA Model)
atmosphere = dlmread('atmosphere.txt');


% AeroCoefficients
%  placeholders only

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

% Define First Stage Characteristics For 1 Module

mP1 = 1600; % First Stage Propellant Loading (Total) (kg)
PCR1 = 16.39; % First Stage Propellant Consumption Rate (Total) (kg/s)
mB1 = 480; % First Stage Burnout Mass (kg)
mAF = 70; % Aero Fuel (kg)
mL = 410; % Landing Mass (kg)

% Define Launch System Characteristics Which are Dependent on no. of First Stage Modules
if N == 4
    A1 = 10; % Reference Area (ARBITRARY PLACEHOLDER) (m^2)
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
r = 6371000; % radius of Earth (m)

dt = 0.1; % Timestep (s)


% Initialise Arrays
beta(1) = deg2rad(90); % Tangential Angle Array (rad)
betadot(1) = 0; % Tangential Anglular Velocity Array (rad/s)
H(1) = 0; % Horizontal Coordinates (m)
V(1) = 0; % Vertical Coordinates (m)
m(1) = (mP1 + + mAF + mL)*N  + mPayload + mCF + mP2 + mB2+ mP3 + mB3; % Mass Array (kg)
t(1) = 0; % Time Array (s)
v_V(1) = 0; % Vertical Velocity Array (m/s)
v_H(1) = 0; % Horizontal Velocity Array (m/s)
v_a = interp1(atmosphere(:,1),atmosphere(:,5),V(1)); % Speed of Sound
M(1) = sqrt(v_H(1)^2+v_V(1)^2)/v_a; % Mach no Array
Cd(1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(1)); % Drag Coefficient Array (N);
D(1) = 0.5* Cd * (v_H(1)^2+v_V(1)^2) * A1 * interp1(atmosphere(:,1),atmosphere(:,4),V(1));
T(1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),0)*N*3; % Thrust Array (kN) (3 Engines on Each ALV-2 First Stage Module)
a_V(1) = (T(1)*10^3-D(1))/m(1)*sin(beta(1)) - 9.81; % Vertical Acceleration Array (m/s^2)
a_H(1) = (T(1)*10^3-D(1))/m(1)*cos(beta(1)); % Horizontal Acceleration Array (m/s^2)



%==========================================================================
%----------------------- First Stage Simulation ---------------------------
%==========================================================================

mParray1(1) = mP1*N; % Initialise Propellant Mass Array (kg)

i = 1; % Temporary Iteration Counter

pitchover = 'no';

while mParray1(i) > 0
% Increment Vehicle Parameters
    
mParray1(i+1) = mParray1(i) - PCR1*N*dt; 

m(i+1) = m(i) - PCR1*N*dt;

H(i+1) = H(i) + v_H(i)*dt;

V(i+1) = V(i) + v_V(i)*dt; 

v_H(i+1) = v_H(i) + a_H(i)*dt;

v_V(i+1) = v_V(i) + a_V(i)*dt;

T(i+1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),V(i+1)/1000) * N*3;

v_a = interp1(atmosphere(:,1),atmosphere(:,5),V(i+1));

M(i+1) = sqrt(v_H(i+1)^2+v_V(i+1)^2)/v_a;

Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 

D(i+1) = 0.5 * Cd(i+1) * (v_H(i+1)^2+v_V(i+1)^2) * A1 * interp1(atmosphere(:,1),atmosphere(:,4),V(i+1));



% Gravity Turn
if t(i) < 30
beta(i+1) = deg2rad(90);
elseif t(i) >= 30 && strcmp(pitchover,'yes') == 0
beta(i+1) = deg2rad(89); % Pitchover Angle (assumed to be instantaneous)
pitchover = 'yes';
elseif t(i) >= 30 && strcmp(pitchover,'yes') == 1
beta(i+1) = beta(i) - 9.81/sqrt(v_H(i+1)^2+v_V(i+1)^2)*sin(deg2rad(90) - beta(i))*dt; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET
end

a_V(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*sin(beta(i+1)) - 9.81; 
a_H(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*cos(beta(i+1));

t(i+1) = t(i) + dt;

i = i+1;
end



plot(H/1000,V/1000)



