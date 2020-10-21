function childNode = transitionState(parentNode, action, expState)
%TRANSITIONSTATE Take a parent node and apply an action to transition this
%to a child node. 

x = parentNode.x + action';

A = expState.A;
W = expState.Q;
R = expState.R;

%Transition Sigma, Equation 6 in Atanasov 2014

%Update

M = H'*inv(V)*H;
Sigma = inv(inv(Sigma) + M)


%Predict
Sigma = A*Sigma*A' + W;

% Sigma = updateSigma();
Sigma = 0;

childNode = createANode(x,Sigma,parentNode);
end

