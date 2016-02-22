function [endPointCost, integrandCost] = SecondStageCost(primal)
%--------------------------------------------------------------
global CONSTANTS
% Minimise Time

% endPointCost    = primal.nodes(end);   
integrandCost   = 0;




% THIRD STAGE GRAVITY TURN
V2 = primal.states(1,:);   
H2 = primal.states(2,:);
v_V2 = primal.states(3,:);
v_H2 = primal.states(4,:);     
m2 = primal.states(5,:); 
t2 =  primal.nodes;

beta2 = primal.controls;


i=1;

m30 = CONSTANTS.mPayload + CONSTANTS.mCF + CONSTANTS.mP3 + CONSTANTS.mB3;

m3(i) = m30;

mParray3(i) = CONSTANTS.mP3;

A3 = 1; 

r = 6371000; % radius of Earth (m)

dt = 0.1; % Timestep (s)
% Initialise Arrays
beta3(1) = beta2(end); % Tangential Angle Array (rad)
H3(1) = H2(end); % Horizontal Coordinates (m)
V3(1) = V2(end); % Vertical Coordinates (m)
m3(1) =  CONSTANTS.mPayload + CONSTANTS.mCF + CONSTANTS.mP3 + CONSTANTS.mB3; % Mass Array (kg)
t3(1) = t2(end); % Time Array (s)
v_V3(1) = v_V2(end); % Vertical Velocity Array (m/s)
v_H3(1) = v_H2(end); % Horizontal Velocity Array (m/s)

T3(1) = .699*4; % Thrust Array (kN) (3 Engines on Each ALV-2 First Stage Module)
a_V3(1) = (T3(1)*10^3)/m3(1)*sin(beta3(1)) - 9.81; % Vertical Acceleration Array (m/s^2)
a_H3(1) = (T3(1)*10^3)/m3(1)*cos(beta3(1)); % Horizontal Acceleration Array (m/s^2)

while mParray3(i) > 0
% Increment Vehicle Parameters
    
mParray3(i+1) = mParray3(i) - CONSTANTS.PCR3*dt; 

m3(i+1) = m3(i) - CONSTANTS.PCR3*dt;

H3(i+1) = H3(i) + v_H3(i)*dt;

V3(i+1) = V3(i) + v_V3(i)*dt; 

v_H3(i+1) = v_H3(i) + a_H3(i)*dt;

v_V3(i+1) = v_V3(i) + a_V3(i)*dt;

T3(i+1) = .699*2;



% Gravity Turn

beta3(i+1) = beta3(i) - 9.81/sqrt(v_H3(i+1)^2+v_V3(i+1)^2)*sin(deg2rad(90) - beta3(i))*dt; % Gravity Turn Initiated  % NO EARTH CURVATURE INCLUDED YET



a_V3(i+1) = (T3(i+1)*10^3)/m3(i+1)*sin(beta3(i+1)) - 6.674e-11.*5.97e24./(V3(i+1) + 6371e3).^2 + v_H3(i+1).^2./(V3(i+1) + 6371e3); 
a_H3(i+1) = (T3(i+1)*10^3)/m3(i+1)*cos(beta3(i+1));

t3(i+1) = t3(i) + dt;

i = i+1;
end
temp_3 = i;


endPointCost    =  abs(V3(end)-400);
