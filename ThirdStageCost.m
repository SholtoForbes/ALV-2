function [endPointCost, integrandCost] = ThirdStageCost(primal)
%--------------------------------------------------------------

% Minimise Time

endPointCost    = primal.nodes(end);  
% endPointCost    = -primal.states(4,end);   
integrandCost   = 0;
