function xdot = SecondStageDynamics(primal)
%--------------------------------------------------------------

global CONSTANTS

Thrust = 3; %kN

% Initialise States
V = primal.states(1,:);   
H = primal.states(2,:);
v = primal.states(3,:);
beta = primal.states(4,:);     
m = primal.states(5,:); 


% Initialise Control
betadot = primal.controls;

%============================================================
%  Equations of motion:
%============================================================
%
Vdot =  v.*sin(beta);
Hdot =  v.*cos(beta);
vdot = Thrust./m; % Currently Ignoring Atmosphere
betadot = betadot; % Need to correct for Earths curvature
mdot = -CONSTANTS.PCR2.*ones(1,CONSTANTS.nodes);

xdot = [Vdot; Hdot; vdot; betadot; mdot];