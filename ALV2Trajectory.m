% ALV-2 Trajectory Simulation 
% Created by Sholto Forbes 27/1/16
clear all
global CONSTANTS
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
r = 6371000; % radius of Earth (m)

dt = 0.1; % Timestep (s)


% Initialise Arrays
beta(1) = deg2rad(90); % Tangential Angle Array (rad)
betadot(1) = 0; % Tangential Anglular Velocity Array (rad/s)
H(1) = 0; % Horizontal Coordinates (m)
V(1) = 0; % Vertical Coordinates (m)
m(1) = (mP1 + mAF + mL)*N  + mPayload + mCF + mP2 + mB2+ mP3 + mB3; % Mass Array (kg)
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

% Utilising a Gravity Turn Manoeuvre 
% Cartesian Coordinates

mParray1(1) = mP1*N; % Initialise Propellant Mass Array (kg)

i = 1; % Temporary Iteration Counter

pitchover = 'no';

t_pitch = 20; % pitchover time

while mParray1(i) > 0
% Increment Vehicle Parameters
    
mParray1(i+1) = mParray1(i) - PCR1*N*dt; 

m(i+1) = m(i) - PCR1*N*dt;

H(i+1) = H(i) + v_H(i)*dt;

V(i+1) = V(i) + v_V(i)*dt; 

v_H(i+1) = v_H(i) + a_H(i)*dt;

v_V(i+1) = v_V(i) + a_V(i)*dt;


if V(i+1) < 48000
T(i+1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),V(i+1)/1000) * N*3; 
else
T(i+1) = interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),48000/1000) * N*3;
end

if V(i+1) < 85000
v_a = interp1(atmosphere(:,1),atmosphere(:,5),V(i+1));
else
v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
end

M(i+1) = sqrt(v_H(i+1)^2+v_V(i+1)^2)/v_a;

Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 

if V(i+1) < 85000
D(i+1) = 0.5 * Cd(i+1) * (v_H(i+1)^2+v_V(i+1)^2) * A1 * interp1(atmosphere(:,1),atmosphere(:,4),V(i+1));
else
D(i+1) = 0;
end

% Gravity Turn
if t(i) < t_pitch
beta(i+1) = deg2rad(90);
elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 0 % Pitchover Time (Arbitrary Currently)
beta(i+1) = deg2rad(89); % Pitchover Angle (assumed to be instantaneous)
pitchover = 'yes';
elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 1
beta(i+1) = beta(i) - 9.81/sqrt(v_H(i+1)^2+v_V(i+1)^2)*sin(deg2rad(90) - beta(i))*dt; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET
end

a_V(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*sin(beta(i+1)) - 6.674e-11.*5.97e24./(V(i+1) + 6371e3).^2 + v_H(i+1).^2./(V(i+1) + 6371e3); 
a_H(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*cos(beta(i+1));

t(i+1) = t(i) + dt;

i = i+1;
end
temp_1 = i;


%==========================================================================
%----------------------- Second + Third Stage Gravity Turn Simulation ---------------------------
%==========================================================================
% % modifying coefficients
% if N == 4
% c1 = 1.8;
% c2 = .2;
% elseif N == 2
% 
% end
% 
% m20 = mPayload + mCF + mP2 + mB2+ mP3 + mB3;
% 
% m(i) = m20;
% 
% mParray2(i) = mP2;
% 
% while mParray2(i) > 0
% % Increment Vehicle Parameters
%     
% mParray2(i+1) = mParray2(i) - PCR2*dt; 
% 
% m(i+1) = m(i) - PCR2*dt;
% 
% H(i+1) = H(i) + v_H(i)*dt;
% 
% V(i+1) = V(i) + v_V(i)*dt; 
% 
% v_H(i+1) = v_H(i) + a_H(i)*dt;
% 
% v_V(i+1) = v_V(i) + a_V(i)*dt;
% 
% if V(i+1) < 48000
% T(i+1) = interp1(SecondStageThrust(:,1),SecondStageThrust(:,6),V(i+1)/1000)*N;
% else
% T(i+1) = interp1(SecondStageThrust(:,1),SecondStageThrust(:,6),48000/1000)*N  ;
% end
% 
% if V(i+1) < 85000
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),V(i+1));
% else
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
% end
% 
% 
% M(i+1) = sqrt(v_H(i+1)^2+v_V(i+1)^2)/v_a;
% 
% Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 
% 
% if V(i+1) < 85000
% D(i+1) = 0.5 * Cd(i+1) * (v_H(i+1)^2+v_V(i+1)^2) * A2 * interp1(atmosphere(:,1),atmosphere(:,4),V(i+1));
% else
% D(i+1) = 0;
% end
% 
% 
% 
% % Gravity Turn
% if t(i) < t_pitch
% beta(i+1) = deg2rad(90);
% elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 0 % Pitchover Time (Arbitrary Currently)
% beta(i+1) = deg2rad(89); % Pitchover Angle (assumed to be instantaneous)
% pitchover = 'yes';
% elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 1
% beta(i+1) = beta(i) - 9.81/sqrt(v_H(i+1)^2+v_V(i+1)^2)*sin(deg2rad(90) - beta(i))*dt*c1; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET
% end
% 
% a_V(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*sin(beta(i+1)) - 6.674e-11.*5.97e24./(V(i+1) + 6371e3).^2 + v_H(i+1).^2./(V(i+1) + 6371e3); 
% a_H(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*cos(beta(i+1));
% 
% t(i+1) = t(i) + dt;
% 
% i = i+1;
% end
% temp_2 = i;
% 
% 
% % third stage
% m30 = mPayload + mCF + mP3 + mB3;
% 
% m(i) = m30;
% 
% mParray3(i) = mP3;
% 
% A3 = 1; 
% 
% while mParray3(i) > 0
% % Increment Vehicle Parameters
%     
% mParray3(i+1) = mParray3(i) - PCR3*dt; 
% 
% m(i+1) = m(i) - PCR3*dt;
% 
% H(i+1) = H(i) + v_H(i)*dt;
% 
% V(i+1) = V(i) + v_V(i)*dt; 
% 
% v_H(i+1) = v_H(i) + a_H(i)*dt;
% 
% v_V(i+1) = v_V(i) + a_V(i)*dt;
% 
% if V(i+1) < 48000
% T(i+1) = interp1(ThirdStageThrust(:,1),ThirdStageThrust(:,6),V(i+1)/1000)*N;
% else
% T(i+1) = interp1(ThirdStageThrust(:,1),ThirdStageThrust(:,6),48000/1000)*N  ;
% end
% 
% if V(i+1) < 85000
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),V(i+1));
% else
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
% end
% 
% 
% M(i+1) = sqrt(v_H(i+1)^2+v_V(i+1)^2)/v_a;
% 
% Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)); 
% 
% if V(i+1) < 85000
% D(i+1) = 0.5 * Cd(i+1) * (v_H(i+1)^2+v_V(i+1)^2) * A3 * interp1(atmosphere(:,1),atmosphere(:,4),V(i+1));
% else
% D(i+1) = 0;
% end
% 
% 
% % Gravity Turn
% if t(i) < t_pitch 
% beta(i+1) = deg2rad(90);
% elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 0 % Pitchover Time (Arbitrary Currently)
% beta(i+1) = deg2rad(89); % Pitchover Angle (assumed to be instantaneous)
% pitchover = 'yes';
% elseif t(i) >= t_pitch && strcmp(pitchover,'yes') == 1
% beta(i+1) = beta(i) - 9.81/sqrt(v_H(i+1)^2+v_V(i+1)^2)*sin(deg2rad(90) - beta(i))*dt*c2; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET
% end
% 
% 
% a_V(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*sin(beta(i+1)) - 6.674e-11.*5.97e24./(V(i+1) + 6371e3).^2 + v_H(i+1).^2./(V(i+1) + 6371e3); 
% a_H(i+1) = (T(i+1)*10^3-D(i+1))/m(i+1)*cos(beta(i+1));
% 
% t(i+1) = t(i) + dt;
% 
% i = i+1;
% end
% temp_3 = i;
% 
% figure(1);
% 
% subplot(5,1,[1 2])
% hold on
% plot(H(1:temp_1)/1000,V(1:temp_1)/1000,'color','g')
% plot(H(temp_1:temp_2)/1000,V(temp_1:temp_2)/1000,'color','r')
% plot(H(temp_2:temp_3)/1000,V(temp_2:temp_3)/1000,'color','b')
% subplot(5,1,3)
% plot(t,m);
% subplot(5,1,4)
% plot(t,v_H);
% subplot(5,1,5)
% plot(t,beta);

% % % %==========================================================================
% % % %----------------------- Second Tangential Steering Simulation ---------------------------
% % % %==========================================================================
% % 
% 
% 
% 
% CONSTANTS.PCR2 = PCR2;
% CONSTANTS.mPayload = mPayload;
% CONSTANTS.mCF = mCF;
% CONSTANTS.mP3 = mP3;
% CONSTANTS.mB3 = mB3;
% CONSTANTS.PCR3 = PCR3;
% 
% 
% % Utilising DIDO to Solve a Linear Tangent Steering Problem
% 
% 
% % bound the time intervals
% %-------------------------
% 
% t0 = 0;   tfMax = 400;  % Maximum Time Interval
% 
% bounds.lower.time = [0 0];           
% bounds.upper.time = [0 tfMax];  
% 
% 
% % Set Initial Conditions
% 
% V0 = V(end);        H0 = 0;     v_V0 = v_V(end);     v_H0 = v_H(end);    m0 = CONSTANTS.mPayload + CONSTANTS.mCF + mP2 + mB2+ CONSTANTS.mP3 + CONSTANTS.mB3;     % Initial Conditions
% 
% %Limiting Conditions and Boundary Constraints
% Vf = 400e03; %Reference Trajectory Altitude (m)
% Hf = 700e03; % Maximum Horizontal Distance (m) (arbitrary)
% v_max =  7.67e03; % Maximum Velocity (m/s) (Orbital Velocity)    
% mf = CONSTANTS.mPayload + mB2 + CONSTANTS.mCF + CONSTANTS.mP3 + CONSTANTS.mB3; % Minimum Mass (kg)
% 
% 
% % Set Boundary Conditions and Limits
% bounds.lower.states = [0.9*V0; H0; -1; 0.9*v_H0; mf];
% bounds.upper.states = [ 1.1*Vf;  Hf; v_max; v_max; m0];
% 
% bounds.lower.controls = 0.;
% bounds.upper.controls = beta(end); % Maximum Angle 
% 
% bounds.lower.events = [V0; H0; v_V0; v_H0; m0; Vf; mf];	
% bounds.upper.events = bounds.lower.events;
% 
% SecondStage.bounds = bounds;
% 
% % No. Nodes To Use
% algorithm.nodes = [40];
% 
% CONSTANTS.nodes = algorithm.nodes;
% 
% % %%Define Guess
% guess.states(1,:) = [V0, 250e03];
% guess.states(2,:) = [H0,  100e03];
% guess.states(3,:) = [v_V0,  0];
% guess.states(4,:) = [v_H0,  v_H0+1000];
% guess.states(5,:) = [m0,  mf];
% guess.controls    = [beta(end), 0];
% guess.time        = [t0, 100];
% 
% algorithm.guess = guess;
% % =======================================================================
% 
% 
% 
% 
% % Define Subroutine Files
% SecondStage.cost      = 'SecondStageCost';
% SecondStage.dynamics  = 'SecondStageDynamics';
% SecondStage.events    = 'SecondStageEvents';
% 
% % Initialise DIDO
% [cost, primal, dual] = dido(SecondStage, algorithm);
% 
% % Plotting ================================================================
% 
% V2 = primal.states(1,:);   
% H2 = primal.states(2,:);
% v_V2 = primal.states(3,:);
% v_H2 = primal.states(4,:);     
% m2 = primal.states(5,:); 
% t2 =  primal.nodes;
% 
% beta2 = primal.controls;
% 
% figure(2)
% subplot(5,1,1)
% plot(H2,V2);
% 
% subplot(5,1,2)
% plot(t2,v_V2);
% 
% subplot(5,1,3)
% plot(t2,v_H2);
% 
% subplot(5,1,4)
% plot(t2,m2);
% 
% subplot(5,1,5)
% plot(t2,beta2);
% 
% figure(3)
% subplot(2,5,[1,5]);
% 
% line(t2, dual.dynamics(1,:),'Color','k', 'LineStyle','-');
% line(t2, dual.dynamics(2,:),'Color','k', 'LineStyle','--');
% line(t2, dual.dynamics(3,:),'Color','k', 'LineStyle','-.');
% line(t2, dual.dynamics(4,:),'Color','k', 'LineStyle',':');
% title('costates')
% xlabel('time');
% ylabel('Costates');
% % axis([0,t2(end),-1,1])
% legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');
% 
% subplot(2,5,[6,10])
% Hamiltonian = dual.Hamiltonian(1,:);
% plot(t2,Hamiltonian,'Color','k');
% % axis([0,t2(end),-1,1])
% title('Hamiltonian')
% 
% 
% % THIRD STAGE GRAVITY TURN
% i=1;
% 
% m30 = CONSTANTS.mPayload + CONSTANTS.mCF + CONSTANTS.mP3 + CONSTANTS.mB3;
% 
% m3(i) = m30;
% 
% mParray3(i) = CONSTANTS.mP3;
% 
% A3 = 1; 
% 
% % Initialise Arrays
% beta3(1) = beta2(end); % Tangential Angle Array (rad)
% H3(1) = H2(end); % Horizontal Coordinates (m)
% V3(1) = V2(end); % Vertical Coordinates (m)
% m3(1) =  CONSTANTS.mPayload + CONSTANTS.mCF + CONSTANTS.mP3 + CONSTANTS.mB3; % Mass Array (kg)
% t3(1) = t2(end); % Time Array (s)
% v_V3(1) = v_V2(end); % Vertical Velocity Array (m/s)
% v_H3(1) = v_H2(end); % Horizontal Velocity Array (m/s)
% v_a3 = interp1(atmosphere(:,1),atmosphere(:,5),V3(1)); % Speed of Sound
% 
% T3(1) = .699*2; % Thrust Array (kN) (3 Engines on Each ALV-2 First Stage Module)
% a_V3(1) = (T3(1)*10^3)/m3(1)*sin(beta3(1)) - 9.81; % Vertical Acceleration Array (m/s^2)
% a_H3(1) = (T3(1)*10^3)/m3(1)*cos(beta3(1)); % Horizontal Acceleration Array (m/s^2)
% 
% while mParray3(i) > 0
% % Increment Vehicle Parameters
%     
% mParray3(i+1) = mParray3(i) - CONSTANTS.PCR3*dt; 
% 
% m3(i+1) = m3(i) - CONSTANTS.PCR3*dt;
% 
% H3(i+1) = H3(i) + v_H3(i)*dt;
% 
% V3(i+1) = V3(i) + v_V3(i)*dt; 
% 
% v_H3(i+1) = v_H3(i) + a_H3(i)*dt;
% 
% v_V3(i+1) = v_V3(i) + a_V3(i)*dt;
% 
% T3(i+1) = .699*N;
% 
% if V3(i+1) < 85000
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),V3(i+1));
% else
% v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
% end
% 
% 
% % Gravity Turn
% 
% beta3(i+1) = beta3(i) - 9.81/sqrt(v_H3(i+1)^2+v_V3(i+1)^2)*sin(deg2rad(90) - beta3(i))*dt; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET
% 
% 
% 
% a_V3(i+1) = (T3(i+1)*10^3)/m3(i+1)*sin(beta3(i+1)) - 6.674e-11.*5.97e24./(V3(i+1) + 6371e3).^2 + v_H3(i+1).^2./(V3(i+1) + 6371e3); 
% a_H3(i+1) = (T3(i+1)*10^3)/m3(i+1)*cos(beta3(i+1));
% 
% t3(i+1) = t3(i) + dt;
% 
% i = i+1;
% end
% temp_3 = i;
% 
% figure(1);
% 
% subplot(5,1,[1 2])
% hold on
% plot(H3/1000,V3/1000,'color','g')
% subplot(5,1,3)
% plot(t3,m3);
% subplot(5,1,4)
% plot(t3,v_H3);
% subplot(5,1,5)
% plot(t3,beta3);
% 



%==========================================================================
%----------------------- Second + Third Stage Simulation ---------------------------
%==========================================================================

CONSTANTS.PCR2 = PCR2;
CONSTANTS.PCR3 = PCR3;

% Utilising DIDO to Solve a Linear Tangent Steering Problem


% bound the time intervals
%-------------------------

t0 = 0;   tfMax = 900;  % Maximum Time Interval

bounds.lower.time = [0 0 0];           
bounds.upper.time = [0 tfMax/2 tfMax];  


% Set Initial Conditions

V0 = V(end);        H0 = 0;     v_V0 = v_V(end);     v_H0 = v_H(end);    m20 = mPayload + mCF + mP2 + mB2+ mP3 + mB3;     % Initial Conditions

m30 = mPayload + mCF + mP3 + mB3; 

%Limiting Conditions and Boundary Constraints
Vf = 400e03; %Reference Trajectory Altitude (m)
Hf = 3000e03; % Maximum Horizontal Distance (m) (arbitrary)
v_max =  7.67e03; % Maximum Velocity (m/s) (Orbital Velocity)    
m2f = mPayload + mB2 + mCF + mP3 + mB3; % Minimum Mass (kg)

m3f = mPayload + mCF + mB3; 

% Set Boundary Conditions and Limits
bounds.lower.states = [0.9*V0; H0; -1; 0.9*v_H0; 0];
bounds.upper.states = [ 1.1*Vf;  Hf; v_max*1.5; v_max*1.5; m20];

bounds.lower.controls = 0.;
bounds.upper.controls = beta(end); % Maximum Angle 

bounds.lower.events = [V0; H0; v_V0; v_H0; m20; Vf; 0; m2f; m30; 0; 0; 0; 0];	
bounds.upper.events = bounds.lower.events;

SecondThirdStage.bounds = bounds;

% No. Nodes To Use
algorithm.nodes = [140 140];

CONSTANTS.nodes = algorithm.nodes;

% Knots Guesses
algorithm.knots.locations    = [t0  300 650];

% %%Define Guess
guess.states(1,:) = [V0,200e03, 400e03];
guess.states(2,:) = [H0,1000e03,  2000e03];
guess.states(3,:) = [v_V0,v_V0/2,  0];
guess.states(4,:) = [v_H0,2500,  8000];
guess.states(5,:) = [m20,m2f,  m3f];
guess.controls    = [beta(end),beta(end)/2, 0];
guess.time        = [t0,300, 650];

algorithm.guess = guess;
% =======================================================================
%  Tell DIDO that all your events are 'hard'; this is a redundant statement 
%========================================================================
algorithm.knots.definitions  = {'hard','hard','hard'};
%==========================================================================


% Define Subroutine Files
SecondThirdStage.cost      = 'SecondThirdStageCost';
SecondThirdStage.dynamics  = 'SecondThirdStageDynamics';
SecondThirdStage.events    = 'SecondThirdStageEvents';

% Initialise DIDO
[cost, primal, dual] = dido(SecondThirdStage, algorithm);

% Plotting ================================================================

V2 = primal.states(1,:);   
H2 = primal.states(2,:);
v_V2 = primal.states(3,:);
v_H2 = primal.states(4,:);     
m2 = primal.states(5,:); 
t2 =  primal.nodes;

beta2 = primal.controls;

figure(2)
subplot(5,1,1)
plot(H2,V2);

subplot(5,1,2)
plot(t2,v_V2);

subplot(5,1,3)
plot(t2,v_H2);

subplot(5,1,4)
plot(t2,m2);

subplot(5,1,5)
plot(t2,beta2);
