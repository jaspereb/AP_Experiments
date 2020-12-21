function cost = getNodeCost(Sigma, x, expState)
%getNodeCost Calculate the cost of a node given Sigma and x for that node.

if(strcmp(expState.costFn,'Trace'))
    cost = trace(Sigma);
    
elseif(strcmp(expState.costFn,'Weighted Trace'))
    SW = expState.SigmaWeighting.*Sigma;
    cost = trace(SW);
    
elseif(strcmp(expState.costFn,'Composite'))
    SW = expState.SigmaWeighting.*Sigma;
    dist = distFromXToY(x); %Need to write this
    cost = expState.costAlpha*trace(SW) + expState.costBeta*dist;
    disp("Not implemented");
    disp(throwanerror);
    
else
    disp("ERROR: Unknown cost function name for getNodeCost");
    return;
end

end

