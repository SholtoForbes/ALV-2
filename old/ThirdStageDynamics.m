function xdot = ThirdStageDynamics(primal)
%--------------------------------------------------------------

% NO DRAG CURRENTLY

global CONSTANTS

Thrust = 700*2.*ones(1,CONSTANTS.nodes); %N  PLACEHOLDER, MAKE VARIABLE


% Initialise States
V = primal.states(1,:);   
H = primal.states(2,:);
v_V = primal.states(3,:);
v_H = primal.states(4,:);     
m = primal.states(5,:); 


% Initialise Control
beta = deg2rad(primal.controls);

%============================================================
%  Equations of motion:
%============================================================
% Need to correct for Earths curvature
Vdot =  v_V;
Hdot =  v_H;

% NEED TO INCLUDE DRAG
v_Vdot = Thrust./m .* sin(beta) - 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3); % vertical acceleration includes centripetal motion and variable gravity
v_Hdot = Thrust./m .* cos(beta);

% betadot = betadot; % Need to correct for Earths curvature

mdot = -CONSTANTS.PCR3.*ones(1,CONSTANTS.nodes);



xdot = [Vdot; Hdot; v_Vdot; v_Hdot; mdot];