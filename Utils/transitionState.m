function childNode = transitionState(parentNode, action, expState)
%TRANSITIONSTATE Take a parent node and apply an action to transition this
%to a child node. Sigma is calculated using the current target pose estimate 
%

x = parentNode.x + action';

A = expState.A;
W = expState.Q;
V = expState.R;

if(strcmp(expState.currExpName,'FVI'))
    y = parentNode.y;
else
    print("NOT IMPLEMENTED!! Need to run EKF step here");
end

% Transition Sigma, Equation 6 in Atanasov 2014
cameraPose = [x;0,0,0,1];
cameraPose = alignCamera(cameraPose,y,expState);

p = cameraPose(1:3);
R = quat2rotm(cameraPose(4:7));

% Update - Calculate H, the linearisation of the observation function about 
% the predicted target pose using Eq 27 from Schlotfeldt 2019. 
K = expState.cameraParams.instrinsics.intrinsicMatrix; %Camera intrinsics
THIS K IS NOT RIGHT, CHECK SHEET DEFINITION AGAINST EQUATION

H = K*piPrime(R'*(y-p))*R';

M = H'*inv(V)*H;
Sigma = inv(inv(Sigma) + M)


% Predict
Sigma = A*Sigma*A' + W;

% Sigma = updateSigma();
Sigma = 0;

childNode = createANode(x,y,Sigma,parentNode);
end

function M = piPrime(N)
    %N is 3x1
    M = (1/(N(3,1)^2))*[N(3,1), 0, -N(1,1); 0, N(3,1), -N(2,1); 0, 0, 0];
end