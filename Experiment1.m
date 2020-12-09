%Experiment 1 compares the estimates from 1000 runs of the straight path
%and 1000 of the diagonal path

clearvars
close all

load('StartState.mat');

straightResults = makeResultsStruct(expState);

% Straight Path Experiment
straightPoses = getPosesStraight(cameraPose,expState);
expState.currExpName = 'Straight Path';
expState.costFn = 'Weighted Trace';
for run = 1:expState.numRuns
    [runState,x,C] = getRandTarget(expState);
    [x_straight,P_straight,z_straight] = runEKF(straightPoses,runState,functH,x,K,C);
    straightResults.x{run} = x_straight;
    straightResults.P{run} = P_straight;
    straightResults.z{run} = z_straight;
    straightResults.targetPose{run} = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end
if(expState.showFigs)
    fig_straight = figure();
    plotState(straightPoses, runState, z_straight, fig_straight);
end

straightResults = calculateResults(straightResults,expState);

% Diagonal Path Experiment
diagonalPoses = getPosesDiagonal(cameraPose,expState);
expState.currExpName = 'Diagonal Path';
expState.costFn = 'Weighted Trace';
for run = 1:expState.numRuns
    [runState,x,C] = getRandTarget(expState);
    [x_diagonal,P_diagonal,z_diagonal] = runEKF(diagonalPoses,runState,functH,x,K,C);
    diagonalResults.x{run} = x_diagonal;
    diagonalResults.P{run} = P_diagonal;
    diagonalResults.z{run} = z_diagonal;
    diagonalResults.targetPose{run} = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end
if(expState.showFigs)
    fig_diag = figure();
    plotState(diagonalPoses, runState, z_straight, fig_diag);
end

diagonalResults = calculateResults(diagonalResults,expState);

clearvars fig_diag fig_straight
save('Exp1Results.mat');

clearvars
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
save('Exp1Figures.mat');
