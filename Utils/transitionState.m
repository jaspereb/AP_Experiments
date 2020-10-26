function childNode = transitionState(parentNode, action, expState)
%TRANSITIONSTATE Take a parent node and applies an action to transition this
%to a child node. Sigma is calculated using the current target pose estimate 
%

%Action is the next camera pose, not an offset

x = action';
Sigma = parentNode.Sigma;

A = expState.A;
W = expState.Q;
V = expState.R;
functH = @ObsFunction;
intrinsics = expState.cameraParams.Intrinsics;

%Turn node state into camera pose
cameraPose = [x;1;0;0;0];
cameraPose = alignCamera(cameraPose,parentNode.y,expState);
p = cameraPose(1:3);
R = quat2rotm(cameraPose(4:7)');

if(strcmp(expState.currExpName,'FVI Offline'))
    y = parentNode.y;
elseif(strcmp(expState.currExpName,'FVI Online'))
    %Run EKF predict & update step for the target state (x in EKF, y in
    %nodes)
    [zHat,C] = calcJac(functH, parentNode.y, cameraPose, expState);
    [u,v] = getDetection(cameraPose, expState);
    z = [u;v];
    
    %Some mismatch in notation between this fn and runEKF
    P = A*parentNode.Sigma*A' + W; 
    
    K = P*C'*inv(V + C*P*C');
    
    yPrime = A*parentNode.y; %Predict target state to current timestep
    yPrime = yPrime + K*(zHat - z);
    
    y = yPrime;
else
    print("ERROR: Unknown experiment name");
    return;
end

% Transition Sigma, Equation 6 in Atanasov 2014
camPoints = toCameraAxes(cameraPose,y);
u = intrinsics.FocalLength(1)*(camPoints(1)/camPoints(3)) + intrinsics.PrincipalPoint(1);
v = intrinsics.FocalLength(2)*(camPoints(2)/camPoints(3)) + intrinsics.PrincipalPoint(2);
if(min([u,v]) <= 0) || (u >= intrinsics.ImageSize(2)) || (v >= intrinsics.ImageSize(1))
    visible = false;
else
    visible = true;
end

if((camPoints(3) < expState.minCamDistance) || (camPoints(3) > expState.maxCamDistance))
    visible = false;
end


if(visible)
    % Update - Calculate H, the linearisation of the observation function about 
    % the predicted target pose using Eq 27 from Schlotfeldt 2019. 
    K = expState.cameraParams.Intrinsics.IntrinsicMatrix; %Camera intrinsics
    K = [K(1,1), 0, K(3,1); 0, K(2,2), K(3,2)];

    H = K*piPrime(R'*(y-p))*R';

    M = H'*inv(V)*H;
    Sigma = inv(inv(Sigma) + M);
end

% Predict
Sigma = A*Sigma*A' + W;
assert(max(isnan(Sigma(:))) == 0);

childNode = createANode(x,y,Sigma,parentNode,visible);
end

function M = piPrime(N)
    %N is 3x1
    M = (1/(N(3,1)^2))*[N(3,1), 0, -N(1,1); 0, N(3,1), -N(2,1); 0, 0, 0];
end