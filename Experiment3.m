%Experiment 2 generates an estimate from 1000 runs of the best path found
%using offline FVI

load('StartState.mat');

THEN EXPERIMENT 3 IS SIMILAR:
DETERMINE THE ACTIONS SET
PROPAGATE SIGMA
REMOVE DOMINATED NODES
ADD NEW NODES TO THE TO-EXPAND SET
CONTINUE WHILE PATH < MAX LENGTH
DETERMINE THE BEST FINAL SIGMA PATH
CHOOSE THE NEXT ACTION IN THIS PATH
RUN THE EKF UPDATE STEP FROM THAT POSITION
RERUN FVI WITH NEW TARGET POSE ESTIMATE

% Determine the best estimated path
FVIPath = getFVIPath(cameraPose,expState);

% Run this 1000 times
expState.currExpName = 'FVI Offline Path';
if(expState.showFigs)
    fig_AP = figure(3);
    plotState(APPoses, expState, fig_AP);
end
[x_AP,P_AP] = runEKF(FVIPath,expState,functH,x,Q,R,A,K,C,P);
