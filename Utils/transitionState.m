function childNode = transitionState(parentNode, action, runState)
%TRANSITIONSTATE Take a parent node and apply an action to transition this
%to a child node. 

x = parentNode.x + action;
% Sigma = updateSigma();
Sigma = 0;

childNode = createANode(x,Sigma,parentNode);
end

