function xdot = SecondStageDynamics(primal)
%--------------------------------------------------------------

global CONSTANTS

Thrust = 20000; %N, THIS IS AN ASSUMPTION

% Initialise States
V = primal.states(1,:);   
H = primal.states(2,:);
v_V = primal.states(3,:);
v_H = primal.states(4,:);     
m = primal.states(5,:); 


% Initialise Control
beta = primal.controls;

%============================================================
%  Equations of motion:
%============================================================
% Need to correct for Earths curvature
Vdot =  v_V;
Hdot =  v_H;
v_Vdot = Thrust./m .* sin(beta) - 9.81;
v_Hdot = Thrust./m .* cos(beta);

% betadot = betadot; % Need to correct for Earths curvature

mdot = -CONSTANTS.PCR2.*ones(1,CONSTANTS.nodes);

xdot = [Vdot; Hdot; v_Vdot; v_Hdot; mdot];