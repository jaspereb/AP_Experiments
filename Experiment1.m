%Experiment 1 compares the estimates from 1000 runs of the straight path
%and 1000 of the diagonal path

clearvars
close all

load('StartState.mat');

straightResults = makeResultsStruct(expState);

% Straight Path Experiment
straightPoses = getPosesStraight(cameraPose,expState);
expState.currExpName = 'Straight Path';
if(expState.showFigs)
    fig_straight = figure();
    plotState(straightPoses, expState, fig_straight);
end
for run = 1:expState.numRuns
    [runState,x,C] = getRandTarget(expState);
    [x_straight,P_straight,z_straight] = runEKF(straightPoses,runState,functH,x,Q,R,A,K,C,P);
    straightResults.x{run} = x_straight;
    straightResults.P{run} = P_straight;
    straightResults.z{run} = z_straight;
    straightResults.targetPose = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end

calculateResults(straightResults,expState);

% Diagonal Path Experiment
diagonalPoses = getPosesDiagonal(cameraPose,expState);
expState.currExpName = 'Diagonal Path';
if(expState.showFigs)
    fig_diag = figure();
    plotState(diagonalPoses, expState, fig_diag);
end
for run = 1:expState.numRuns
    [runState,x,C] = getRandTarget(expState);
    [x_diagonal,P_diagonal,z_diagonal] = runEKF(diagonalPoses,runState,functH,x,Q,R,A,K,C,P);
    diagonalResults.x{run} = x_diagonal;
    diagonalResults.P{run} = P_diagonal;
    diagonalResults.z{run} = z_diagonal;
    diagonalResults.targetPose = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end

save('sampleResults.mat');
calculateResults(diagonalResults,expState);
