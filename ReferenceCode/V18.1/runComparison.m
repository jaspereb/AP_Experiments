%Run a comparison between selecting random actions, selecting greedy
%actions and using A*

%This runs each of these for a specific number of steps, then compares the 
% -Final log det E
% -Area under the log det E curve
% -Final yHat accuracy based on the actual observations
% -Path taken
% -Actual E and yHat at each step

clearvars
close all

%% Experiment Settings
% expSet.numActions = 27; %Number of action choices at each step
% expSet.actions = 10;
% expSet.initialState = [0,0,0,0.707,0,0,-0.707]';
% expSet.yTrue = [1.0, 1.0, 4.0]';
% expSet.yHat = expSet.yTrue + normrnd(0,0.5,3,1);
% expSet.Ezero = eye(3);
% expSet.stepSize = 0.3; %Distance per action step
% expSet.goalThresh = expSet.stepSize*2;
% expSet.sensorNoiseStd = 5.0; %Std dev of the sensor noise in pixels
% expSet.gamma = 1.0; %Tuning factor for explore/exploit
% expSet.heuristicTimesteps = 4; %The heuristic lookahead timesteps

expSet.actions = 18;
expSet.initialState = [0,0,0,0.707,0,0,-0.707]';
expSet.yTrue = [0.0, 0.0, 4.0]';
expSet.yHat = expSet.yTrue;% + normrnd(0,0.5,3,1);
expSet.Ezero = 0.001*eye(3);
expSet.stepSize = 0.3; %Distance per action step
expSet.goalThresh = expSet.stepSize*2;
expSet.sensorNoiseStd = 5.0; %Std dev of the sensor noise in pixels
expSet.gamma = 1.0; %Tuning factor for explore/exploit
expSet.heuristicTimesteps = 4; %The heuristic lookahead timesteps
expSet.W = [1,0,0;0,1,0;0,0,1];
expSet.costType = 'trace'; %Options are 'logdet','trace'
expSet.constDist = true; %Generate actions of constant distance
expSet.numActions = 27; %Number of action choices at each step
expSet.minSensorDist = 0.2; %Minimum sensor range, below this no info is gained
expSet.maxSensorDist = 5; %Max sensor range, above this no info is gained
expSet.dominatedDistance = 0.5*expSet.stepSize; %Distance threshold to consider a node dominated
expSet.visIntermediatePaths = true; %Show each path as it is expanded

%% Results Objects
randResults = [];
greedyResults = [];
aStarResults = [];

run chooseRandom;
clearvars -except expSet randResults greedyResults aStarResults
run chooseGreedy;
clearvars -except expSet randResults greedyResults aStarResults
% run chooseAStar;
% clearvars -except expSet randResults greedyResults aStarResults

% load('sampleResult.mat');

%% Check Results
%Plot cost over time
figure(2);
hold on
plot(1:expSet.actions,randResults.cost,'r-');
plot(1:expSet.actions,greedyResults.cost,'g-');
% plot(1:expSet.actions,aStarResults.cost,'b-');

% paths = {randResults.path,greedyResults.path,aStarResults.path};
paths = {randResults.path,greedyResults.path};
plotPath(expSet.yTrue, expSet.yHat, paths);

%% Run Estimator on Path
yHat_rand = runEstimator(expSet, randResults.path);
yHat_greedy = runEstimator(expSet, greedyResults.path);
% yHat_aStar = runEstimator(expSet, aStarResults.path);

randResults.error = mean(abs(yHat_rand - expSet.yTrue));
greedyResults.error = mean(abs(yHat_greedy - expSet.yTrue));
% aStarResults.error = mean(abs(yHat_aStar - expSet.yTrue));

fprintf("Final yHat errors are: \n Random %f \n Greedy %f \n A* %f \n", ...
    randResults.error, greedyResults.error, aStarResults.error);




