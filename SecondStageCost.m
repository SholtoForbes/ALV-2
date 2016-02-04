function [endPointCost, integrandCost] = SecondStageCost(primal)
%--------------------------------------------------------------

% Minimise Time

endPointCost    = primal.nodes(end);  
% endPointCost    = -primal.states(4,end);   
integrandCost   = 0;
