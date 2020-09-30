function yHat = runEstimator(expSet,path)
%RUNESTIMATOR Takes in a sensor path, sensor noise, initial target position 
% estimate, true target position.
%Returns an estimate of the target position using a 2D camera model.

%Code adapted from PIPELINE>VisualServoing>Optimisation

intrinsics = getD435Intrinsics(1);
K = [intrinsics.fx,0,intrinsics.ppx;...
    0,intrinsics.fy,intrinsics.ppy];

observations = []; %Pixel observations

%For each step in the path, generate a noisy observation
for t = 1:size(path,2)
    x = path(:,t);
    R = x(4:7);
    R = quat2rotm(R');
    piMat = R'*(expSet.yTrue-x(1:3));
    piMat = (1/piMat(3))*piMat;
    
    z_true = K*piMat; %This is [u,v] order
    
    observations(:,t) = z_true + [normrnd(0,expSet.sensorNoiseStd);normrnd(0,expSet.sensorNoiseStd)];
end

optFun = @(yHat)estimatorCostFn(observations, path, K, yHat);

% options = optimoptions('fmincon','Algorithm', 'sqp', 'Display','off','MaxFunctionEvaluations',50000000, 'MaxIterations', 10000);
% [solvedParams,minCost] = fmincon(optFun, state, [], [], [], [], [], [], [], options);

options = optimset('PlotFcns',@optimplotfval);
state = fminsearch(optFun, expSet.yHat, options);

yHat = state;

end
