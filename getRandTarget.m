function [expState,x,C] = getRandTarget(expState)
%GETRANDTARGET Does all of the initialisation required to start a single
%stochastic filter run. Including sampling the target position. 

% Add Initial Noise
expState.targetPose = expState.initialTargetPose + ...
    [normrnd(0,expState.targetXYNoise);normrnd(0,expState.targetXYNoise);normrnd(0,expState.targetZNoise)];

cameraPose = expState.initialPose;

%Turn off cam max/min limits for this step
expStateCp = expState;
expStateCp.minCamDistance = 0;
expStateCp.maxCamDistance = inf;
[u,v] = getDetection(cameraPose, expStateCp);

% Estimate initial state using first detection, assume that we know roughly 
% the distance to trellis
x(:,1) = estInitialState(u,v,expState.trellisDist,cameraPose,expState);

functH = @ObsFunction;
[zHat,C{1}] = calcJac(functH, x(:,1), cameraPose, expState);

%Check calcJac has returned a correct observation
assert(zHat(1) == u);
assert(zHat(2) == v);

end

