function xdot = SecondStageDynamics(primal)
%--------------------------------------------------------------

global CONSTANTS



% Initialise States
V = primal.states(1,:);   
H = primal.states(2,:);
v_V = primal.states(3,:);
v_H = primal.states(4,:); 

% m = primal.states(5,:); 

t = primal.nodes;

global m2

m2(1) = CONSTANTS.m20;
Thrust(1) = 3000*4; %N

for i = 2:CONSTANTS.nodes
    if m2(i-1) > CONSTANTS.m30
    m2(i) = m2(i-1) - CONSTANTS.PCR2*(t(i)-t(i-1));
    Thrust(i) = 3000*4; 
    else
    m2(i) = m2(i-1) - CONSTANTS.PCR3*(t(i)-t(i-1));
    Thrust(i) = 700*2; 
    end
end

% Initialise Control
beta = primal.controls;

%============================================================
%  Equations of motion:
%============================================================
% Need to correct for Earths curvature
Vdot =  v_V;
Hdot =  v_H;
v_Vdot = Thrust./m2 .* sin(beta) - 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3); % vertical acceleration includes centripetal motion and variable gravity
v_Hdot = Thrust./m2 .* cos(beta);

% betadot = betadot; % Need to correct for Earths curvature

% mdot = -CONSTANTS.PCR2.*ones(1,CONSTANTS.nodes);

% xdot = [Vdot; Hdot; v_Vdot; v_Hdot; mdot];
xdot = [Vdot; Hdot; v_Vdot; v_Hdot];