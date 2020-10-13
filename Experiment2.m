%Experiment 2 generates an estimate from 1000 runs of the best path found
%using offline FVI

load('StartState.mat');

% Determine the best estimated path
FVIPath = getFVIPath(cameraPose,expState);

% Run this 1000 times
expState.currExpName = 'FVI Offline Path';
if(expState.showFigs)
    fig_AP = figure(3);
    plotState(APPoses, expState, fig_AP);
end
[x_AP,P_AP] = runEKF(FVIPath,expState,functH,x,Q,R,A,K,C,P);
