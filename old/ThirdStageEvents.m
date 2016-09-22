function eventConditions = ThirdStageEvents(primal)

%-----------------------------------------------%
global CONSTANTS

V0 = primal.states(1,1);    Vf = primal.states(1,end);
H0 = primal.states(2,1);
v_V0 = primal.states(3,1);     v_Vf = primal.states(3,end);
v_H0 = primal.states(4,1);     v_Hf = primal.states(4,end);   
m0 = primal.states(5,1);    mf = primal.states(5,end); 



%--------------------------------------------------------------------------

%% pre-allocate 8 event conditions
eventConditions = zeros(8,1);
%%
eventConditions(1) = V0;
eventConditions(2) = H0;
eventConditions(3) = v_V0;
eventConditions(4) = v_H0;
eventConditions(5) = m0;
eventConditions(6) = Vf;
eventConditions(7) = v_Vf;
eventConditions(8) = v_Hf ;


