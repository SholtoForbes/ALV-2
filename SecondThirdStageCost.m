function [endPointCost, integrandCost] = SecondStageCost(primal)
%--------------------------------------------------------------

% Minimise Time

endPointCost    = primal.nodes(end);   
integrandCost   = 0;
