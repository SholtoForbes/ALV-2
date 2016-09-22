function eventConditions = SecondStageEvents(primal)

%-----------------------------------------------%
global CONSTANTS

V0 = primal.states(1,1);    Vf = primal.states(1,end);
H0 = primal.states(2,1);
v_V0 = primal.states(3,1);     v_Vf = primal.states(3,end);
v_H0 = primal.states(4,1);     
m0 = primal.states(5,1);    



%--------------------------------------------------------------------------
% Collect all the left and right limit points
Left    = primal.indices.left;      
Right   = primal.indices.right;

preSeparation_V = primal.states(1, Left);   postSeparation_V = primal.states(1, Right);
preSeparation_H = primal.states(2, Left);   postSeparation_H = primal.states(2, Right);
preSeparation_v_V = primal.states(3, Left);   postSeparation_v_V = primal.states(3, Right);
preSeparation_v_H = primal.states(4, Left);   postSeparation_v_H = primal.states(4, Right);
preSeparation_m = primal.states(5, Left);   postSeparation_m = primal.states(5, Right);

stage1FuelUsed = m0 - preSeparation_m;
%% pre-allocate 8 event conditions
eventConditions = zeros(13,1);
%%
eventConditions(1) = V0;
eventConditions(2) = H0;
eventConditions(3) = v_V0;
eventConditions(4) = v_H0;
eventConditions(5) = m0;
eventConditions(6) = Vf;
eventConditions(7) = v_Vf;

eventConditions(8) = preSeparation_m;
eventConditions(9) = postSeparation_m;

eventConditions(10) = postSeparation_V - preSeparation_V;
eventConditions(11) = postSeparation_H - preSeparation_H;
eventConditions(12) = postSeparation_v_V - preSeparation_v_V;
eventConditions(13) = postSeparation_v_H - preSeparation_v_H;

