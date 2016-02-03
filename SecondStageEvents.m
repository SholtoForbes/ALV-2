function eventConditions = SecondStageEvents(primal)

%-----------------------------------------------%
global CONSTANTS

V0 = primal.states(1,1);    Vf = primal.states(1,end);
H0 = primal.states(2,1);
v0 = primal.states(3,1);
beta0 = primal.states(4,1);     betaf = primal.states(4,end);
m0 = primal.states(5,1);    

%--------------------------------------------------------------------------

%% pre-allocate 8 event conditions
eventConditions = zeros(7,1);
%%
eventConditions(1) = V0;
eventConditions(2) = H0;
eventConditions(3) = v0;
eventConditions(4) = beta0;
eventConditions(5) = m0;
eventConditions(6) = Vf;
eventConditions(7) = betaf;


